// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5

/**
 * @implements REQ-CAPI-HOST-EXIT-001
 * @covers AC-1 AC-2 AC-4
 *
 * @implements REQ-CAPI-TEARDOWN-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-CACHE-001
 * @covers AC-1 AC-3 AC-5
 *
 * @implements REQ-CAPI-DETACH-001
 * @covers AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-TIMESTEP-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-CONTACT-003
 * @covers AC-3
 *
 * @implements REQ-CAPI-TIMESTEP-002
 * @covers AC-1
 *
 * @implements REQ-CAPI-THREADS-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-5 AC-6 AC-14 AC-16
 */

/**
 * @implements REQ-CAPI-TASK-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10
 *
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-1 AC-4 AC-5 AC-6 AC-7
 *
 * @implements REQ-CAPI-STEP-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-OVD-NAMES-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-LOG-001
 * @covers AC-5 AC-6
 *
 * @implements REQ-CAPI-STRING-001
 * @covers AC-2 AC-3
 *
 * @implements REQ-PYTHON-LIFECYCLE-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-OMNIPVD-001
 * @covers AC-2 AC-3
 *
 * @implements REQ-CAPI-OMNIPVD-CAPTURE-001
 * @covers AC-2 AC-3 AC-4 AC-5 AC-7 AC-8
 *
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11 AC-12
 *
 * @implements REQ-CAPI-READPOOL-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 */

/**
 * @implements REQ-CAPI-SDFVIEW-001
 * @covers AC-1
 *
 * @implements REQ-CAPI-CUDA-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-CPU-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-DETACH-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-OVSTAGE-ATTACH-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-1 AC-2 AC-3 AC-6
 *
 * @implements REQ-CAPI-OVSTAGE-UPDATE-002
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-ATTACH-OWNER-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-INSTANCE-SCOPE-001
 * @covers AC-1 AC-3 AC-6
 *
 * @implements REQ-CAPI-PROFILER-001
 * @covers AC-1 AC-2 AC-3 AC-5
 *
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3 AC-8
 *
 * @implements REQ-CAPI-THREADING-001
 * @covers AC-2 AC-3 AC-4 AC-5 AC-6
 *
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-9 AC-10 AC-11
 *
 * @implements REQ-CAPI-DIRECTGPU-DIAG-001
 * @covers AC-1 AC-3
 *
 * @implements REQ-CAPI-COMPILED-CACHE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 */

/**
 * @implements REQ-CAPI-LANE-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-REENTRY-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-ASYNC-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
 */

#include <omnipvd/PxOmniPvd.h>
#include <PxPhysicsAPI.h>
#include <omni/physx/OmniPvdCaptureResolver.h>
#include <omni/physx/PrimPathGrammar.h>
#include <omni/physx/IPhysx.h>
#include "internal/CpuFeatureCheck.h"
#include "LogManager.hpp"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "cuda_shim/CudaShim.h"
#include <omni/physx/AttachOptions.h>
#include <omni/physx/IOvxPhysicsWrite.h>
#include <omni/physx/PhysXRuntime.h>
#include <omni/physx/RuntimeError.h>
#include "UsdSchemaPaths/UsdSchemaPaths.h"
#include "internal/sdk/PhysxSchemaRegistration.hpp"
#include "internal/sdk/TaskContext.hpp"
#include <ovstage/ovstage_population.h>
using ovphysx::TaskContext;

#include <carb/ClientUtils.h>
#include <carb/logging/Logger.h>
#include <algorithm>
#include <cassert>
#include <charconv>
#include <climits>
#include <cmath>
#include <cstring>
#include <vector>
#include <atomic>
#include <shared_mutex>
#include <cinttypes> // For PRIu64 portable format specifier

// Platform-specific includes (Windows/Linux dynamic loading)
#include "internal/sdk/PlatformIncludes.hpp"
#include "internal/Profiling.h"


// PhysX simulation interface
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IOvxPhysicsRead.h>
#include <omni/physx/IPhysxSettings.h>
// PhysX foundation interface (GPU availability check)
#include <omni/physx/IPhysxFoundation.h>

#include <omni/physics/tensors/TensorApi.h>
#include <omni/core/BuiltIn.h>
#include <omni/core/OmniInit.h>
// Settings for sharing state
#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/IThreadPool.h>
// Extension interface for tensor plugin initialization
#include <omni/ext/IExt.h>
#include <omni/physics/tensors/TensorApi.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <iomanip>
#include <inttypes.h>
#include <mutex>
#include <filesystem>
#include <vector>
#include <algorithm>
#include <limits>
#if CARB_PLATFORM_LINUX
#include <execinfo.h>
#endif
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physics/tensors/ISdfShapeView.h>
// Internal sidecar API + loader (shared between main library and replicator)
#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"
// Global instance map. shared_ptr storage keeps an instance alive for callers
// that still hold it after erase. Its static destruction at process exit only drops
// references: it never stops or joins an instance worker, which only destroy and
// ovphysx_shutdown() do (see InstanceExecutor::create() for the worker's lifetime).
std::unordered_map<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>> g_instances;

// Guards g_instances. Lookups take shared_lock, insert/erase take unique_lock.
std::shared_mutex g_instances_mutex;

namespace
{

// The one process-wide serial sequence behind every ovphysx-owned opaque object
// handle: instances, output sessions, tensor bindings, contact bindings and SDF views.
//
// It is process-wide on purpose (NVBug 6504951). Independent per-kind counters
// share the same numeric space, so an instance handle and that instance's first
// tensor binding would both be 1, and a binding handle from a destroyed instance
// would match the first binding of the next one. One never-reused sequence keeps
// the handle kinds numerically unique for the life of the process. Handles
// stay opaque uint64_t and 0 stays the invalid sentinel.
//
// Deliberately a single internally-linked constant-initialized atomic with no
// mutex, pointer or singleton, so there is no static-initialization or
// destruction order to reason about. Do not turn this into a function-local
// static or a registry.
std::atomic<uint64_t> g_nextOpaqueObjectHandle{ 1 };

} // namespace

uint64_t ovphysx::internal::allocateOpaqueObjectHandle() noexcept
{
    return allocateOpaqueObjectHandle(g_nextOpaqueObjectHandle);
}

uint64_t ovphysx::internal::exchangeNextOpaqueObjectHandle(uint64_t next) noexcept
{
    return g_nextOpaqueObjectHandle.exchange(next);
}

struct TaskWaitStorage
{
    TaskWaitStorage() : tasks(1) {}

    // The leading live receipt is a private allocation header, never a task.
    // Keeping it in the same array makes recovery through error_tasks[-1] valid.
    // Copy pointer bytes through its integer field without alignment or aliasing casts.
    void set_owner(std::shared_ptr<TaskWaitStorage>* owner) noexcept
    {
        static_assert(sizeof(owner) <= sizeof(tasks.front().instance), "Wait-result owner must fit in the header");
        std::memcpy(&tasks.front().instance, &owner, sizeof(owner));
    }

    static std::shared_ptr<TaskWaitStorage>* owner_for(const ovphysx_enqueue_result_t* failures) noexcept
    {
        std::shared_ptr<TaskWaitStorage>* owner = nullptr;
        if (failures)
            std::memcpy(&owner, &failures[-1].instance, sizeof(owner));
        return owner;
    }

    ovphysx_enqueue_result_t* error_tasks() noexcept { return tasks.data() + 1; }
    size_t error_count() const noexcept { return tasks.size() - 1; }

    std::vector<ovphysx_enqueue_result_t> tasks;
    std::vector<std::string> errors;
};

// ---------------------------------------------------------------------------
// Instance executor integration: Carbonite-aware waits and worker jobs.
// ---------------------------------------------------------------------------

namespace
{

// Carbonite tasking, captured after the PhysX plugins started it. Executor waits use
// these hooks to suspend a Carbonite task instead of blocking its worker thread,
// which the simulation stepper may need. Cleared before Carbonite shuts down.
std::atomic<carb::tasking::ITasking*> g_executorTasking{nullptr};

uint64_t executorCurrentTask()
{
    carb::tasking::ITasking* tasking = g_executorTasking.load(std::memory_order_acquire);
    return tasking ? static_cast<uint64_t>(tasking->getTaskContext()) : 0;
}

void executorWaitTask(const std::atomic<uint32_t>* word, uint32_t expected, uint64_t timeoutNs)
{
    carb::tasking::ITasking* tasking = g_executorTasking.load(std::memory_order_acquire);
    if (!tasking)
        return;
    if (timeoutNs == UINT64_MAX)
    {
        tasking->futexWait(*word, expected);
        return;
    }
    const uint64_t bounded = std::min<uint64_t>(timeoutNs, static_cast<uint64_t>(INT64_MAX));
    tasking->futexWaitFor(*word, expected, std::chrono::nanoseconds(static_cast<int64_t>(bounded)));
}

void executorWakeTasks(const std::atomic<uint32_t>* word)
{
    carb::tasking::ITasking* tasking = g_executorTasking.load(std::memory_order_acquire);
    if (tasking)
        tasking->futexWakeup(*word, UINT_MAX);
}

// pinToCurrentThread() reports whether the task was already pinned; only a pin this
// wait added is removed afterwards.
bool executorPinTask()
{
    carb::tasking::ITasking* tasking = g_executorTasking.load(std::memory_order_acquire);
    return tasking && tasking->getTaskContext() != carb::tasking::kInvalidTaskContext &&
           !tasking->pinToCurrentThread();
}

void executorUnpinTask()
{
    carb::tasking::ITasking* tasking = g_executorTasking.load(std::memory_order_acquire);
    if (tasking)
        tasking->unpinFromCurrentThread();
}

const ovphysx::ExecutorWaitHooks g_executorWaitHooks{
    executorCurrentTask, executorWaitTask, executorWakeTasks, executorPinTask, executorUnpinTask};

// Installs the hooks once carb.tasking is running. Never starts the plugin: worker
// pool sizing must stay configurable until the PhysX plugins start it.
void installExecutorWaitHooks()
{
    carb::Framework* framework = carb::getFramework();
    carb::tasking::ITasking* tasking =
        framework ? framework->tryAcquireExistingInterface<carb::tasking::ITasking>() : nullptr;
    if (!tasking)
        return;
    g_executorTasking.store(tasking, std::memory_order_release);
    ovphysx::setExecutorWaitHooks(&g_executorWaitHooks);
}

void removeExecutorWaitHooks()
{
    // Final shutdown reaches this only with no live instances, after every
    // executor and task-aware waiter is gone, so no wait can observe removal.
    ovphysx::setExecutorWaitHooks(nullptr);
    g_executorTasking.store(nullptr, std::memory_order_release);
}

} // namespace

namespace
{
void relaxWorkerCaptureMode() noexcept;
bool exchangeCaptureMode(CUstreamCaptureMode& mode) noexcept;
} // namespace

ovphysx::InstanceJob::InstanceJob(const char* api, ovphysx_instance_handle_t handle,
                                  std::shared_ptr<InstanceData> instance, Body body)
    : m_api(api), m_handle(handle), m_instance(std::move(instance)), m_body(std::move(body))
{
}

ovphysx::InstanceJob::InstanceJob(const char* api, ovphysx_instance_handle_t handle,
                                  std::shared_ptr<InstanceData> instance)
    : m_api(api), m_handle(handle), m_instance(std::move(instance))
{
}

void ovphysx::InstanceJob::fail(const char* cause) noexcept
{
    if (failed)
        return;
    failed = true;
    try
    {
        m_cause = cause ? cause : "";
    }
    catch (...)
    {
        m_cause.clear();
    }
}

void ovphysx::InstanceJob::run()
{
    // Before the CUDA work of its first job, a worker relaxes its stream-capture mode;
    // in hard CPU-only mode it makes no CUDA call at all. A waiting thread may run a job
    // instead (InstanceExecutor::runUnstartedJobs()): it relaxes its mode for that job
    // only, so a capture that thread runs refuses none of the job's CUDA calls. Without a
    // worker thread the job is part of the call that submitted it and keeps that thread's
    // mode, as a synchronous call does.
    const bool onWorker = m_instance->executor->isWorkerThread();
    if (onWorker && !m_instance->worker_capture_mode_set)
    {
        m_instance->worker_capture_mode_set = true;
        relaxWorkerCaptureMode();
    }
    struct CallerCaptureMode
    {
        CUstreamCaptureMode mode = CU_STREAM_CAPTURE_MODE_RELAXED;
        bool exchanged = false;
        ~CallerCaptureMode()
        {
            if (exchanged)
                exchangeCaptureMode(mode);
        }
    } callerMode;
    if (!onWorker && m_instance->executor->hasWorker())
        callerMode.exchanged = exchangeCaptureMode(callerMode.mode);
    ovphysx::profiling::ApiProfileZone zone(
        carb::profiler::kCaptureMaskDefault,
        CARB_PROFILE_BEGIN(carb::profiler::kCaptureMaskDefault, "ovphysx job %s", m_api));
    omni::physx::RuntimeErrorScope runtimeError;
    // The thread's last error describes this job only.
    success();
    try
    {
        execute();
    }
    catch (const std::exception& error)
    {
        fail(error.what());
    }
    catch (...)
    {
        fail("Unknown exception during task execution");
    }
    if (!failed)
        return;
    if (m_cause.empty() && runtimeError.message()[0])
    {
        try
        {
            m_cause = runtimeError.message();
        }
        catch (...)
        {
            m_cause.clear();
        }
    }
    try
    {
        failure = std::string(m_api) + " (task " + std::to_string(taskIndex) + "): " +
                  (m_cause.empty() ? std::string("Task execution failed") : m_cause);
    }
    catch (...)
    {
        failure.clear();
    }
}

ovphysx_enqueue_result_t ovphysx::submitInstanceJob(InstanceData& instance, ovphysx_instance_handle_t handle,
                                                   std::unique_ptr<InstanceJob> job)
{
    return runWithoutWorker(instance, queueInstanceJob(instance, handle, std::move(job)));
}

ovphysx_enqueue_result_t ovphysx::runWithoutWorker(InstanceData& instance, const ovphysx_enqueue_result_t& receipt)
{
    if (receipt.status != OVPHYSX_API_SUCCESS || instance.executor->hasWorker())
        return receipt;
    instance.executor->runSubmittedJobs(receipt.op_index);
    // The job's own errors belong to its task, not to the accepted call.
    success();
    return receipt;
}

ovphysx_enqueue_result_t ovphysx::queueInstanceJob(InstanceData& instance, ovphysx_instance_handle_t handle,
                                                  std::unique_ptr<InstanceJob> job)
{
    if (!instance.executor)
        return set_enqueue_error(OVPHYSX_API_INVALID_STATE, "Instance worker is not available");
    std::unique_ptr<ovphysx::InstanceExecutor::Job> queued(std::move(job));
    ovphysx::InstanceExecutor::SubmitStatus status = ovphysx::InstanceExecutor::SubmitStatus::kStopped;
    const uint64_t index = instance.executor->submit(queued, status);
    switch (status)
    {
    case ovphysx::InstanceExecutor::SubmitStatus::kAccepted:
        success();
        return {OVPHYSX_API_SUCCESS, index, handle};
    case ovphysx::InstanceExecutor::SubmitStatus::kQueueFull:
        return set_enqueue_error(OVPHYSX_API_INVALID_STATE,
                                 "Instance work queue is full; wait for earlier tasks before submitting more");
    case ovphysx::InstanceExecutor::SubmitStatus::kIndexExhausted:
        return set_enqueue_error(OVPHYSX_API_ERROR, "Task index exhausted");
    case ovphysx::InstanceExecutor::SubmitStatus::kStopped:
    default:
        return set_enqueue_error(OVPHYSX_API_INVALID_STATE,
                                 "Instance worker is stopped; destroy the instance");
    }
}

// Serializes the process-global attach-time GPU selector with scene attachment
// so simultaneous attach calls from different handles cannot overwrite it while
// PhysX consumes it.
std::mutex g_gpuAttachMutex;

// Guards the recording state below. Code that writes the owner holds this lock and
// g_gpuAttachMutex, so code that holds either one reads it. Nothing holds this lock for
// long, and nothing is locked under it but an instance's job queue, so attach and detach
// submissions, which take only this lock, never wait for another instance's running
// attach or detach.
static std::mutex g_recordingMutex;

// The public owner of the shared runtime's one active OmniPVD stream. This is
// separate from stage ownership: a peer handle may start recording the one
// attached stage. Recording transitions share g_gpuAttachMutex with attach and
// detach so a stream cannot be rebound while its stage is being torn down.
static ovphysx_instance_handle_t g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;

// Recording starts that have not finished yet, guarded by g_recordingMutex alone. Attach
// and detach submitted while a recording is active or starting run on the calling thread
// (see lifecycle_runs_on_caller()).
static uint32_t g_recordingStartsPending = 0;

// Sets the recording owner. Call with g_gpuAttachMutex held.
static void setRecordingOwner(ovphysx_instance_handle_t owner)
{
    std::lock_guard<std::mutex> lock(g_recordingMutex);
    g_omniPvdRecordingOwner = owner;
}

// Lifecycle work of different instances never overlaps: attach and detach jobs hold
// this lock while they run, and so do attach and detach that run on the calling thread
// while a recording is active, and destroy while it tears an instance down. It comes
// after every instance lane a call holds and before every other lock (see the lock order
// in ovphysxSDK.hpp). Nothing waits for a lane, or joins a worker, while holding it: a
// lifecycle job holds only its own lane, a call that needs every lane takes them all
// first, and destroy takes it after joining the instance worker. Like g_gpuAttachMutex it
// is a plain mutex: a Carbonite task that blocks on it, such as a destroy while another
// instance's attach runs, keeps its pool thread for that time, unlike a lane wait.
static std::mutex g_lifecycleMutex;

namespace
{

// Every CUDA driver call ovphysx made to run long calls on instance workers.
std::atomic<uint64_t> g_workerCudaCalls{0};

// The CUDA interface for running long calls on instance workers, or null in hard
// CPU-only mode, which never touches CUDA, and without a usable driver.
omni::physx::IOptionalCuda* workerCuda() noexcept
{
    if (isProcessGpuDisabled())
        return nullptr;
    omni::physx::IOptionalCuda* cuda = omni::physx::runtime::tryGetOptionalCudaInterface();
    return cuda && cuda->cudaAvailable && cuda->cudaAvailable() ? cuda : nullptr;
}

// Sets the calling instance worker's CUDA stream-capture mode to relaxed, so that CUDA
// does not refuse its potentially unsafe calls, and the runtime's on this thread, while
// another thread captures a graph in global mode (a refused call would invalidate that
// capture). Work on the legacy default stream still conflicts with a capture of a
// blocking stream.
void relaxWorkerCaptureMode() noexcept
{
    if (!workerCuda())
        return;
    CUstreamCaptureMode mode = CU_STREAM_CAPTURE_MODE_RELAXED;
    g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
    const CUresult result = omni::physx::cudaShim::cuThreadExchangeStreamCaptureMode_(&mode);
    if (result != CUDA_SUCCESS)
        CARB_LOG_WARN("[ovphysx] Instance worker kept its CUDA stream-capture mode (CUDA error %d)",
                      static_cast<int>(result));
}

// Exchanges the calling thread's CUDA stream-capture mode with mode, and returns whether it
// did: false in hard CPU-only mode and without a usable driver, with no CUDA call.
bool exchangeCaptureMode(CUstreamCaptureMode& mode) noexcept
{
    if (!workerCuda())
        return false;
    g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
    return omni::physx::cudaShim::cuThreadExchangeStreamCaptureMode_(&mode) == CUDA_SUCCESS;
}

// Whether a CUDA graph capture of a blocking stream holds the legacy default stream of
// the calling thread's current context, which any work on that stream would invalidate.
// The query itself leaves the capture intact. Captures of non-blocking streams are not
// seen. False in hard CPU-only mode and without a usable driver, with no CUDA call, and
// without a current context, where the capture query would fail as an API error.
bool legacyStreamHeldByCapture() noexcept
{
    omni::physx::IOptionalCuda* cuda = workerCuda();
    if (!cuda || !cuda->ctxGetCurrent)
        return false;
    uintptr_t context = 0;
    g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
    if (!cuda->ctxGetCurrent(&context, nullptr) || !context)
        return false;
    bool capturing = false;
    int status = 0;
    g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
    return !cuda->streamIsCapturing(0, &capturing, &status) && status == CUDA_ERROR_STREAM_CAPTURE_IMPLICIT;
}

// The CUDA context current on the calling thread, for an attach job that the instance
// worker or a waiting caller runs in it; zero when none is current.
uintptr_t submittingCudaContext() noexcept
{
    omni::physx::IOptionalCuda* cuda = workerCuda();
    if (!cuda || !cuda->ctxGetCurrent)
        return 0;
    uintptr_t context = 0;
    g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
    return cuda->ctxGetCurrent(&context, nullptr) ? context : 0;
}

// Reproduces a queued attach's submitting CUDA context on the worker or a waiter
// taking over the job. Captured absence temporarily removes the executing thread's
// entire context stack; synchronous recording attaches keep their caller's context.
class WorkerCudaContextScope
{
public:
    WorkerCudaContextScope(uintptr_t context, bool queuedAttach)
    {
        if (!queuedAttach)
            return;
        m_cuda = workerCuda();
        if (!m_cuda)
        {
            m_failed = context != 0;
            return;
        }
        if (!m_cuda->ctxGetCurrent || !m_cuda->ctxPushCurrent || !m_cuda->ctxPopCurrent)
        {
            m_failed = true;
            return;
        }
        if (context)
        {
            g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
            m_pushed = m_cuda->ctxPushCurrent(context, &m_status);
            m_failed = !m_pushed;
            return;
        }
        try
        {
            for (;;)
            {
                uintptr_t current = 0;
                g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
                if (!m_cuda->ctxGetCurrent(&current, &m_status))
                {
                    m_failed = true;
                    return;
                }
                if (!current)
                {
                    m_restoreEmpty = true;
                    return;
                }
                // Reserve storage before changing the caller's stack. If allocation
                // fails, restore every context already removed before propagating.
                m_detached.push_back(current);
                g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
                if (!m_cuda->ctxPopCurrent(nullptr, &m_status))
                {
                    m_detached.pop_back();
                    m_failed = true;
                    return;
                }
            }
        }
        catch (...)
        {
            restoreDetached();
            throw;
        }
    }
    ~WorkerCudaContextScope()
    {
        if (m_pushed)
        {
            g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
            (void)m_cuda->ctxPopCurrent(nullptr, nullptr);
        }
        restoreDetached();
    }
    WorkerCudaContextScope(const WorkerCudaContextScope&) = delete;
    WorkerCudaContextScope& operator=(const WorkerCudaContextScope&) = delete;

    bool failed() const noexcept { return m_failed; }
    int status() const noexcept { return m_status; }

private:
    void restoreDetached() noexcept
    {
        bool restored = true;
        // Device-ordinal attachment can bind a new context while the stack is
        // empty. Remove it before putting the executing thread's stack back.
        while (m_restoreEmpty)
        {
            uintptr_t current = 0;
            g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
            if (!m_cuda->ctxGetCurrent(&current, nullptr))
            {
                restored = false;
                break;
            }
            if (!current)
                break;
            g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
            if (!m_cuda->ctxPopCurrent(nullptr, nullptr))
            {
                restored = false;
                break;
            }
        }
        m_restoreEmpty = false;
        while (!m_detached.empty())
        {
            g_workerCudaCalls.fetch_add(1, std::memory_order_relaxed);
            if (!m_cuda->ctxPushCurrent(m_detached.back(), nullptr))
                restored = false;
            m_detached.pop_back();
        }
        if (!restored)
            CARB_LOG_ERROR("Could not restore the executing thread's CUDA context stack after attach");
    }

    omni::physx::IOptionalCuda* m_cuda = nullptr;
    std::vector<uintptr_t> m_detached;
    bool m_pushed = false;
    bool m_restoreEmpty = false;
    bool m_failed = false;
    int m_status = 0;
};

} // namespace

uint64_t ovphysx_worker_cuda_calls() noexcept
{
    return g_workerCudaCalls.load(std::memory_order_relaxed);
}

void ovphysx_run_under_lifecycle_lock(void (*fn)(void*), void* user)
{
    std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
    fn(user);
}

// The InstanceData::test_execution of instances created from now on; only the test hook
// ovphysx_set_test_execution_for_test_internal() changes it.
static std::atomic<ovphysx::TestExecution> g_testExecution{ovphysx::TestExecution::kNatural};

void ovphysx_set_test_execution(ovphysx::TestExecution mode) noexcept
{
    g_testExecution.store(mode, std::memory_order_relaxed);
}

// Process-wide CPU-only mode. Set via ovphysx_set_cpu_mode(true) before any instances
// are created. When true, IPhysxFoundation::setCpuMode(true) is applied at first attach,
// preventing any CUDA driver contact for the process lifetime.
static std::atomic<bool> g_forceCpuMode{false};
// Startup-only global configuration becomes immutable after the first instance
// successfully creates the shared runtime.
static std::atomic<bool> g_runtimeConfigStarted{false};

// OVPHYSX_DISABLE_GPU is latched at ovphysx_initialize() and stays sticky until
// shutdown. Outside that interval the env is read live, so a pre-init
// get_cpu_mode() cannot snapshot a missing variable and miss a later setenv.
static std::atomic<bool> g_envGpuDisabledLatched{false};
static std::atomic<bool> g_envGpuDisabledValue{false};

static bool readEnvGpuDisabledLive()
{
    return std::getenv("OVPHYSX_DISABLE_GPU") != nullptr;
}

static void latchEnvGpuDisabledFromEnvironment()
{
    g_envGpuDisabledValue.store(readEnvGpuDisabledLive(), std::memory_order_release);
    g_envGpuDisabledLatched.store(true, std::memory_order_release);
}

static void clearEnvGpuDisabledLatch()
{
    g_envGpuDisabledLatched.store(false, std::memory_order_release);
}

static bool isEnvGpuDisabled()
{
    if (g_envGpuDisabledLatched.load(std::memory_order_acquire))
        return g_envGpuDisabledValue.load(std::memory_order_acquire);
    return readEnvGpuDisabledLive();
}

// True when this process must never touch the GPU: CPU-only mode forced via
// ovphysx_set_cpu_mode(true) or OVPHYSX_DISABLE_GPU active. Whether a usable CUDA
// device is present is a separate check through the CUDA shim. Declared in
// ovphysxSDK.hpp so other translation units gate GPU-only work on the same predicate.
bool isProcessGpuDisabled()
{
    // g_forceCpuMode can still be toggled by ovphysx_set_cpu_mode() before instances
    // exist, so it is loaded live.
    return g_forceCpuMode.load(std::memory_order_acquire) || isEnvGpuDisabled();
}
std::atomic<uint32_t> g_unloadSequenceCounter{0};

enum class UnloadCaller : uint8_t
{
    kUnknown = 0,
    kAPI,
    kDestroy,
    kShutdownHook,
};

thread_local UnloadCaller g_threadUnloadCaller = UnloadCaller::kUnknown;

// Mirror of omni::physx::SceneMultiGPUMode (omni.physx plugin-internal enum).
// Defined locally because the canonical header (SceneMultiGPUMode.h) is not on
// ovphysx's include path. Values map 1:1 to /physics/sceneMultiGPUMode settings.
static constexpr int32_t kMultiGPU_Disabled  = 0; // single GPU (eDisabled)
static constexpr int32_t kMultiGPU_All       = 1; // all GPUs, round-robin (eAll)
static constexpr int32_t kMultiGPU_SkipFirst = 2; // all except first GPU (eSkipFirst)

// ---------------------------------------------------------------------------
// active_cuda_gpus helpers
// ---------------------------------------------------------------------------

// Parse a comma-separated GPU ordinal string into a sorted vector.
// Empty input returns {0} as an internal parse placeholder. The create path
// preserves empty input as "no ovphysx ordinal override" and distinguishes it
// from an explicit "0" before storing the parsed ordinals.
// On error, writes a message to errbuf and returns an empty vector.
static std::vector<int32_t> parseActiveCudaGpus(const char* str, size_t len,
                                                 char* errbuf, size_t errsize)
{
    if (!str || len == 0)
        return {0};

    std::vector<int32_t> result;
    const char* p   = str;
    const char* end = str + len;

    while (p < end)
    {
        while (p < end && (*p == ' ' || *p == '\t')) ++p;
        if (p >= end) break;

        bool negative = false;
        if (*p == '-') { negative = true; ++p; }

        if (p >= end || !isdigit(static_cast<unsigned char>(*p)))
        {
            snprintf(errbuf, errsize, "active_cuda_gpus: expected integer, got '%.*s'",
                     static_cast<int>(end - p < 16 ? end - p : 16), p);
            return {};
        }

        int64_t val64 = 0;
        while (p < end && isdigit(static_cast<unsigned char>(*p)))
        {
            val64 = val64 * 10 + (*p++ - '0');
            if (val64 > std::numeric_limits<int32_t>::max())
            {
                snprintf(errbuf, errsize, "active_cuda_gpus: ordinal value out of range");
                return {};
            }
        }
        const int32_t val = negative ? -static_cast<int32_t>(val64) : static_cast<int32_t>(val64);
        if (val < 0 && val != -1)
        {
            snprintf(errbuf, errsize,
                     "active_cuda_gpus: %d is not a valid ordinal (only -1 is allowed as a special value)", val);
            return {};
        }
        result.push_back(val);

        while (p < end && (*p == ' ' || *p == '\t')) ++p;
        if (p < end)
        {
            if (*p == ',')
            {
                ++p;
                while (p < end && (*p == ' ' || *p == '\t')) ++p;
                if (p >= end)
                {
                    snprintf(errbuf, errsize, "active_cuda_gpus: trailing comma not allowed");
                    return {};
                }
                continue;
            }
            snprintf(errbuf, errsize, "active_cuda_gpus: unexpected character '%c'", *p);
            return {};
        }
    }

    if (result.empty())
        return {0};

    std::sort(result.begin(), result.end());

    // -1 (PhysX auto-select) is only valid as a single ordinal
    if (result.size() > 1)
    {
        for (int32_t v : result)
        {
            if (v < 0)
            {
                snprintf(errbuf, errsize,
                         "active_cuda_gpus: negative ordinal %d cannot be combined with other ordinals", v);
                return {};
            }
        }
    }

    // Only -1 is a valid negative ordinal (PhysX auto-select).
    if (result.size() == 1 && result[0] < -1)
    {
        snprintf(errbuf, errsize,
                 "active_cuda_gpus: invalid negative ordinal %d (only -1 is supported for PhysX auto-select)",
                 result[0]);
        return {};
    }

    for (size_t i = 1; i < result.size(); ++i)
    {
        if (result[i] == result[i - 1])
        {
            snprintf(errbuf, errsize, "active_cuda_gpus: duplicate ordinal %d", result[i]);
            return {};
        }
    }

    return result;
}

// Map a sorted ordinal list to a SceneMultiGPUMode constant.
// deviceCount is the total number of CUDA devices on the machine.
// Returns kMultiGPU_All, kMultiGPU_SkipFirst, or -1 on error.
static int32_t determineMultiGPUMode(const std::vector<int32_t>& ordinals, int32_t deviceCount,
                                     char* errbuf, size_t errsize)
{
    const auto n = static_cast<int32_t>(ordinals.size());

    // eAll: contiguous [0 .. deviceCount-1]
    if (n == deviceCount && ordinals[0] == 0)
    {
        bool ok = true;
        for (int32_t i = 0; i < n; ++i) if (ordinals[i] != i) { ok = false; break; }
        if (ok) return kMultiGPU_All;
    }

    // eSkipFirst: contiguous [1 .. deviceCount-1]
    if (n == deviceCount - 1 && ordinals[0] == 1)
    {
        bool ok = true;
        for (int32_t i = 0; i < n; ++i) if (ordinals[i] != i + 1) { ok = false; break; }
        if (ok) return kMultiGPU_SkipFirst;
    }

    // Build ordinal list string for the error message
    std::string ordStr;
    for (int32_t i = 0; i < n; ++i)
    {
        if (i) ordStr += ',';
        ordStr += std::to_string(ordinals[i]);
    }
    snprintf(errbuf, errsize,
             "active_cuda_gpus=[%s] is not supported. Supported patterns: single ordinal, "
             "all %d GPUs (0..%d), or all except first (1..%d). Got %d CUDA device(s).",
             ordStr.c_str(), deviceCount, deviceCount - 1, deviceCount - 1, deviceCount);
    return -1;
}

// Range- and multi-GPU-pattern-validate a parsed, non-empty active_cuda_gpus
// ordinal list against the live CUDA device count. Callers gate on
// isProcessGpuDisabled()/isCudaAvailable() and a non-empty request string
// before calling. Shared by instance creation and ovphysx_set_config() so
// both surfaces enforce the same rule.
static bool validateActiveCudaGpuOrdinals(const std::vector<int32_t>& ordinals,
                                          char* errbuf, size_t errsize)
{
    int count = 0;
    const CUresult deviceCountResult = omni::physx::cudaShim::cuDeviceGetCount_(&count);
    const bool haveCount = deviceCountResult == CUDA_SUCCESS && count > 0;

    // -1 means "PhysX auto-select" and is intentionally left unchecked.
    const int32_t firstOrdinal = ordinals[0];
    if (firstOrdinal >= 0 && haveCount && firstOrdinal >= count)
    {
        snprintf(errbuf, errsize,
                 "active_cuda_gpus: ordinal %d is out of range (CUDA device count: %d)",
                 firstOrdinal, count);
        return false;
    }

    if (ordinals.size() > 1 && haveCount && determineMultiGPUMode(ordinals, count, errbuf, errsize) < 0)
        return false;

    return true;
}

// Apply the per-instance GPU selection immediately before PhysX attaches a
// scene. PhysX reads these process-global settings lazily while creating the
// first GPU scene. The caller must hold g_gpuAttachMutex and keep it held until
// the immediately following attachStage/attachOvstage call returns.
static void applyAttachTimeGpuSelection(const InstanceData& instance,
                                        carb::Framework* framework,
                                        const char* attachPath)
{
    // active_cuda_gpus is meaningless when GPU use is disabled. Avoid the
    // device-count probe as part of the process-wide no-CUDA-touch contract.
    const std::vector<int32_t>& ordinals = instance.active_cuda_ordinals;
    if (ordinals.empty() || isProcessGpuDisabled() || !framework)
        return;

    carb::settings::ISettings* settings =
        framework->tryAcquireInterface<carb::settings::ISettings>();
    if (!settings)
        return;

    const int32_t gpuIndex = ordinals[0];
    settings->setInt("/physics/cudaDevice", gpuIndex);
    CARB_LOG_INFO("[ovphysx] Applying CUDA device %d before %s", gpuIndex, attachPath);

    if (ordinals.size() <= 1)
    {
        settings->setInt("/physics/sceneMultiGPUMode", kMultiGPU_Disabled);
        return;
    }

    int count = 0;
    const CUresult deviceCountResult = omni::physx::cudaShim::cuDeviceGetCount_(&count);
    if (deviceCountResult != CUDA_SUCCESS || count <= 0)
        return;

    char modeErr[384] = {};
    const int32_t multiGPUMode = determineMultiGPUMode(ordinals, count, modeErr, sizeof(modeErr));
    if (multiGPUMode >= 0)
    {
        settings->setInt("/physics/sceneMultiGPUMode", multiGPUMode);
        return;
    }

    // Create-time validation normally rejects unsupported patterns. Preserve a
    // diagnostic here in case device discovery was unavailable during create.
    CARB_LOG_WARN("[ovphysx] active_cuda_gpus multi-GPU mode not applied: %s", modeErr);
}

// Serialize ovphysx_create_instance() to protect shared Carbonite/plugin init.
static std::mutex g_createInstanceMutex;

// The C API has one SDK lifecycle owner. Companion synchronization and tables
// are members of its companion, never independent global synchronization state.
namespace {
struct SdkLifecycle
{
    std::atomic<bool> initialized{false};
    std::unique_ptr<CompanionRuntime> companion{new CompanionRuntime};
    ~SdkLifecycle()
    {
        // Explicit close is the unload gate. If callers abandon live services,
        // keep their code/data mapped rather than invoke callbacks during static
        // teardown after Carbonite has already started unloading.
        if (companion->hasServiceLeases() || !companion->sessions().entries.empty())
            companion.release();
    }
};
SdkLifecycle g_sdkLifecycle;
}
CompanionRuntime& companionRuntime() { return *g_sdkLifecycle.companion; }

struct StageLifecycleEntry
{
    int64_t stageId;
    std::atomic<bool> unloading;
    std::atomic<bool> detached;

    explicit StageLifecycleEntry(int64_t id = 0)
        : stageId(id)
        , unloading(false)
        , detached(false)
    {
    }

    StageLifecycleEntry(const StageLifecycleEntry& other)
        : stageId(other.stageId)
        , unloading(other.unloading.load(std::memory_order_relaxed))
        , detached(other.detached.load(std::memory_order_relaxed))
    {
    }

    StageLifecycleEntry& operator=(const StageLifecycleEntry& other)
    {
        if (this != &other)
        {
            stageId = other.stageId;
            unloading.store(other.unloading.load(std::memory_order_relaxed));
            detached.store(other.detached.load(std::memory_order_relaxed));
        }
        return *this;
    }
};

std::mutex g_stageLifecycleMutex;
std::unordered_map<int64_t, StageLifecycleEntry> g_stageLifecycleEntries;

namespace
{
    const char* toString(UnloadCaller caller)
    {
        switch (caller)
        {
            case UnloadCaller::kUnknown:
                return "Unknown";
            case UnloadCaller::kAPI:
                return "API";
            case UnloadCaller::kDestroy:
                return "Destroy";
            case UnloadCaller::kShutdownHook:
                return "ShutdownHook";
        }
        return "UnknownCaller";
    }

    class ScopedUnloadCaller
    {
    public:
        explicit ScopedUnloadCaller(UnloadCaller caller)
            : m_prev(g_threadUnloadCaller)
        {
            g_threadUnloadCaller = caller;
        }

        ~ScopedUnloadCaller()
        {
            g_threadUnloadCaller = m_prev;
        }

    private:
        UnloadCaller m_prev;
    };

    class StageUnloadGuard
    {
    public:
        explicit StageUnloadGuard(StageLifecycleEntry* entry)
            : m_entry(entry)
        {
        }

        ~StageUnloadGuard()
        {
            if (m_entry)
            {
                m_entry->unloading.store(false);
            }
        }

        void disarm()
        {
            m_entry = nullptr;
        }

    private:
        StageLifecycleEntry* m_entry;
    };

    StageLifecycleEntry* registerStageLifecycleEntry(int64_t stageId)
    {
        std::lock_guard<std::mutex> lock(g_stageLifecycleMutex);
        auto [it, inserted] = g_stageLifecycleEntries.emplace(stageId, StageLifecycleEntry{stageId});
        it->second.detached.store(false);
        it->second.unloading.store(false);
        if (inserted)
        {
            CARB_LOG_INFO("[STAGE_REGISTRY] Tracking stage %" PRId64, stageId);
        }
        else
        {
            CARB_LOG_INFO("[STAGE_REGISTRY] Stage %" PRId64 " re-registered", stageId);
        }
        return &it->second;
    }

    StageLifecycleEntry* getStageLifecycleEntry(int64_t stageId)
    {
        std::lock_guard<std::mutex> lock(g_stageLifecycleMutex);
        auto it = g_stageLifecycleEntries.find(stageId);
        return (it != g_stageLifecycleEntries.end()) ? &it->second : nullptr;
    }

    void markStageDetached(int64_t stageId, const char* reason)
    {
        StageLifecycleEntry* entry = nullptr;
        {
            std::lock_guard<std::mutex> lock(g_stageLifecycleMutex);
            auto it = g_stageLifecycleEntries.find(stageId);
            if (it != g_stageLifecycleEntries.end())
            {
                it->second.detached.store(true);
                it->second.unloading.store(false);
                entry = &it->second;
            }
        }
        if (entry)
        {
            CARB_LOG_INFO("[STAGE_REGISTRY] Stage %" PRId64 " marked detached (%s)", stageId, reason);
        }
        else
        {
            CARB_LOG_WARN("[STAGE_REGISTRY] Stage %" PRId64 " was already untracked when marking detached (%s)", stageId, reason);
        }
    }

    void logCallStackForUnload(uint32_t unloadSeq)
    {
#if CARB_PLATFORM_LINUX
        void* frames[32];
        int count = ::backtrace(frames, static_cast<int>(sizeof(frames) / sizeof(frames[0])));
        if (count <= 0)
        {
            CARB_LOG_WARN("[UNLOAD #%u] Unable to capture call stack", unloadSeq);
            return;
        }
        char** symbols = ::backtrace_symbols(frames, count);
        if (!symbols)
        {
            CARB_LOG_WARN("[UNLOAD #%u] backtrace_symbols failed", unloadSeq);
            return;
        }
        CARB_LOG_WARN("[UNLOAD #%u] Call stack (most recent call first):", unloadSeq);
        for (int i = 0; i < count; ++i)
        {
            CARB_LOG_WARN("    %s", symbols[i]);
        }
        std::free(symbols);
#else
        CARB_LOG_WARN("[UNLOAD #%u] Call stack capture not supported on this platform", unloadSeq);
#endif
    }

    void logCallerSymbol(uint32_t unloadSeq)
    {
#if CARB_PLATFORM_LINUX
        void* addr = __builtin_return_address(0);
        Dl_info info{};
        if (addr && ::dladdr(addr, &info) && info.dli_sname)
        {
            CARB_LOG_WARN("[UNLOAD #%u] Return address %p (%s @ %s)", unloadSeq, addr, info.dli_sname, info.dli_fname);
        }
        else
        {
            CARB_LOG_WARN("[UNLOAD #%u] Return address %p (symbol resolution unavailable)", unloadSeq, addr);
        }
#else
        CARB_LOG_WARN("[UNLOAD #%u] Return address capture not supported on this platform", unloadSeq);
#endif
    }

} // end anonymous namespace (temporarily closed for ovphysx_ensure_physics_attached / omni_sdk_physx_simulate_instance)

// Runtime status is internal. Preserve the existing ovphysx error surface.
static ovphysx_api_status_t checkRuntimeResult(omni::physx::RuntimeResult result, const char* operation)
{
    if (result == omni::physx::RuntimeResult::eSuccess)
        return OVPHYSX_API_SUCCESS;
    return set_error(OVPHYSX_API_ERROR, std::string(operation) + " failed (runtime status " +
                     std::to_string(static_cast<uint32_t>(result)) + ")").status;
}

static bool recordingActive(omni::physx::IPhysxSimulation* simulation)
{
    bool recording = false;
    return simulation && simulation->getOmniPvdRecordingStatus &&
        simulation->getOmniPvdRecordingStatus(&recording) == omni::physx::RuntimeResult::eSuccess && recording;
}

// Lazily calls attachStage() for the current stage if not yet done, then runs the
// initial PhysX scene parse (simulate(0,0) + fetchResults) so TensorAPI can discover
// prims. Deferred until the caller has drained ovstage edits. Called from simulate(),
// warmup(), and create_tensor_binding(). Runs under the caller's instance lane and
// never takes it again.
ovphysx_api_status_t ovphysx_ensure_physics_attached(ovphysx_instance_handle_t handle)
{
    // The instance map lock covers the lookup only: the lane guards the rest, and a lazy
    // attach below publishes the attachment under that lock exclusively.
    const std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return OVPHYSX_API_ERROR;
    assert(!instance->executor || instance->executor->heldByCurrent());

    // Fast path: already fully initialised
    if (instance->physics_attached.load(std::memory_order_acquire) &&
        instance->initial_parse_done.load(std::memory_order_acquire))
        return OVPHYSX_API_SUCCESS;

    int64_t stageId = instance->attachedStageId;
    if (stageId == 0 && instance->ovstage_attached)
    {
        omni::physx::IPhysxSimulation* physxSim =
            instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
        if (!physxSim)
            return OVPHYSX_API_ERROR;

        if (!instance->initial_parse_done.load(std::memory_order_acquire)) {
            try {
                if (checkRuntimeResult(physxSim->simulate(instance->attachHandle, 0.0f, 0.0f), "Initial simulate") != OVPHYSX_API_SUCCESS)
                    return OVPHYSX_API_ERROR;
                if (checkRuntimeResult(physxSim->fetchResults(instance->attachHandle), "fetchResults") != OVPHYSX_API_SUCCESS)
                    return OVPHYSX_API_ERROR;
                CARB_LOG_INFO("[PHYSICS] Initial ovstage scene parse completed");
            } catch (const std::exception& e) {
                // Leave physics_attached / initial_parse_done unset so the next call
                // retries the parse instead of proceeding on an inconsistent runtime.
                CARB_LOG_WARN("[PHYSICS] Initial ovstage scene parse exception: %s", e.what());
                return OVPHYSX_API_ERROR;
            } catch (...) {
                CARB_LOG_WARN("[PHYSICS] Initial ovstage scene parse failed (unknown exception)");
                return OVPHYSX_API_ERROR;
            }
            instance->initial_parse_done.store(true, std::memory_order_release);
        }

        // Mark attached only after a successful initial parse.
        instance->physics_attached.store(true, std::memory_order_release);
        return OVPHYSX_API_SUCCESS;
    }
    if (stageId == 0)
        return OVPHYSX_API_SUCCESS;

    auto* framework = carb::getFramework();
    if (!framework)
        return OVPHYSX_API_ERROR;

    omni::physx::IPhysxSimulation* physxSim =
        instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;

    // Attach PhysX to the stage if not done yet.
    if (!instance->physics_attached.load(std::memory_order_acquire)) {
        // All attach calls hold g_gpuAttachMutex so simultaneous calls from different
        // handles cannot overwrite /physics/cudaDevice while PhysX consumes it.
        std::unique_lock<std::mutex> attachLock(g_gpuAttachMutex);

        // Re-check under the lock. A concurrent ensure_physics_attached() for this handle
        // may have completed the attach while waiting on attachLock, and attachStage()
        // must not run a second time on an already-attached stage.
        if (!instance->physics_attached.load(std::memory_order_acquire)) {
            // Apply process-wide CPU-only mode before PhysX creates a CUDA context manager.
            // IPhysxFoundation::setCpuMode(true) is sticky for the process lifetime and gates
            // all CUDA driver calls inside the static foundation service.
            if (g_forceCpuMode.load(std::memory_order_acquire))
            {
                if (omni::physx::IPhysxFoundation* foundation =
                        omni::physx::runtime::tryGetPhysxFoundationInterface())
                {
                    foundation->setCpuMode(true);
                }
            }

            applyAttachTimeGpuSelection(*instance, framework, "attachStage");

            bool physx_ok = false;
            if (physxSim) {
                const omni::physics::tensors::AttachHandle newAttach = physxSim->attachStage(stageId, {});
                {
                    std::unique_lock<std::shared_mutex> liveLock(g_instances_mutex);
                    instance->attachHandle = newAttach;
                }
                physx_ok = newAttach != omni::physics::kNoAttach;
                if (physx_ok)
                    omni::physx::ovxDebugRenderNoteAttach(handle, newAttach, 0);
            }

            if (!physx_ok) {
                CARB_LOG_ERROR("[PHYSICS] attachStage() failed for stage %" PRId64
                               " (physx=%s)", stageId,
                               physxSim ? "false" : "unavailable");
                return OVPHYSX_API_ERROR;
            }

            if (!ovphysx_object_change_attach_instance(handle, instance->attachHandle))
            {
                physxSim->detachStage(instance->attachHandle);
                ovphysx_object_change_detached_instance(handle);
                omni::physx::ovxDebugRenderNoteAttach(handle, 0, 1);
                std::unique_lock<std::shared_mutex> liveLock(g_instances_mutex);
                instance->attachHandle = omni::physics::kNoAttach;
                return OVPHYSX_API_ERROR;
            }
            instance->physics_attached.store(true, std::memory_order_release);
            CARB_LOG_INFO("[PHYSICS] Lazy attachStage() completed for stage %" PRId64, stageId);
        }
    }

    // PhysX needs a simulate()+fetchResults() cycle to discover articulations,
    // joints, etc. from the attached scene.
    if (!instance->initial_parse_done.load(std::memory_order_acquire)) {
        if (physxSim) {
            try {
                if (checkRuntimeResult(physxSim->simulate(instance->attachHandle, 0.0f, 0.0f), "Initial simulate") != OVPHYSX_API_SUCCESS)
                    return OVPHYSX_API_ERROR;
                if (checkRuntimeResult(physxSim->fetchResults(instance->attachHandle), "fetchResults") != OVPHYSX_API_SUCCESS)
                    return OVPHYSX_API_ERROR;
                CARB_LOG_INFO("[PHYSICS] Initial scene parse completed for stage %" PRId64, stageId);
            } catch (const std::exception& e) {
                CARB_LOG_WARN("[PHYSICS] Initial scene parse exception: %s", e.what());
            } catch (...) {
                CARB_LOG_WARN("[PHYSICS] Initial scene parse failed (unknown exception)");
            }
        }
        instance->initial_parse_done.store(true, std::memory_order_release);
    }

    return OVPHYSX_API_SUCCESS;
}

// The first half of a step under the caller's hold on the instance lane: the lazy
// physics attach and initial parse, then simulate(). Returns SUCCESS, or ERROR with the
// last error set. dispatched reports whether the runtime accepted the step: an accepted
// step must be fetched, even when simulate() reported an error, which is then the
// step's first error.
static ovphysx_api_status_t dispatch_simulation(InstanceData& instance, ovphysx_instance_handle_t handle,
                                                float elapsedTime, float currentTime, bool& dispatched)
{
    dispatched = false;
    if (elapsedTime < 0.0f) {
        CARB_LOG_ERROR("[PHYSICS SIMULATION] ERROR: Invalid elapsedTime: %f", elapsedTime);
        return OVPHYSX_API_ERROR;
    }

    if (currentTime < 0.0f) {
        CARB_LOG_ERROR("[PHYSICS SIMULATION] ERROR: Invalid currentTime: %f", currentTime);
        return OVPHYSX_API_ERROR;
    }

    // Ensure attachStage() has been called after ovstage ingestion.
    // Must happen before physxSim->simulate().
    const ovphysx_api_status_t attach_status = ovphysx_ensure_physics_attached(handle);
    if (attach_status != OVPHYSX_API_SUCCESS)
        return attach_status;

    omni::physx::IPhysxSimulation* physxSim =
        instance.carbonite ? instance.carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim) {
        CARB_LOG_ERROR("[PHYSICS SIMULATION] ERROR: PhysX simulation interface not available");
        return OVPHYSX_API_ERROR;
    }

    // Only the lane holder changes the attachment, so it is read without the map lock.
    const omni::physics::tensors::AttachHandle attach = instance.attachHandle;
    try {
        omni::physx::RuntimeErrorScope runtimeError;
        if (checkRuntimeResult(physxSim->simulate(attach, elapsedTime, currentTime), "simulate") != OVPHYSX_API_SUCCESS)
            return OVPHYSX_API_ERROR;
        dispatched = true;
        if (attach != omni::physics::tensors::kNoAttach)
            instance.first_step_done.store(true, std::memory_order_release);
        if (runtimeError.message()[0])
            return set_error(OVPHYSX_API_ERROR, runtimeError.message()).status;
        return OVPHYSX_API_SUCCESS;
    } catch (const std::exception& error) {
        return set_error(OVPHYSX_API_ERROR, error.what()).status;
    } catch (...) {
        return set_error(OVPHYSX_API_ERROR, "Unknown exception during simulate").status;
    }
}

static void swap_thread_error(ThreadLocalError& other) noexcept
{
    ThreadLocalError& current = tls_error();
    current.last_error.swap(other.last_error);
    const ovphysx_string_t fallback = current.fallback_error;
    current.fallback_error = other.fallback_error;
    other.fallback_error = fallback;
}

// The second half of a dispatched step under the lane: fetchResults(), which also
// delivers the step's post-step events and callbacks on this thread. failed tells that
// the dispatch already reported the step's first error, which then stays the last
// error. Returns SUCCESS, or ERROR with the last error set.
static ovphysx_api_status_t fetch_simulation(InstanceData& instance, bool failed)
{
    // fetchResults delivers application callbacks on this thread. Preserve an
    // earlier simulate failure before a callback can clear or replace TLS error
    // text; the dispatch failure remains the step's first failure.
    const bool preserveDispatchFailure = failed;
    ThreadLocalError dispatchError;
    if (preserveDispatchFailure)
        swap_thread_error(dispatchError);
    const auto fail = [&failed](const char* message) {
        if (!failed)
            set_error(OVPHYSX_API_ERROR, message);
        failed = true;
    };
    omni::physx::IPhysxSimulation* physxSim =
        instance.carbonite ? instance.carbonite->getPhysxSimulation() : nullptr;
    const omni::physics::tensors::AttachHandle attach = instance.attachHandle;
    if (!physxSim) {
        fail("PhysX simulation interface not available");
    } else if (attach != omni::physics::tensors::kNoAttach) {
        try {
            omni::physx::RuntimeErrorScope runtimeError;
            if (physxSim->fetchResults(attach) != omni::physx::RuntimeResult::eSuccess)
                fail("fetchResults rejected by the runtime");
            if (runtimeError.message()[0])
                fail(runtimeError.message());
        } catch (const std::exception& error) {
            fail(error.what());
        } catch (...) {
            fail("Unknown exception during fetchResults");
        }
    }
    if (preserveDispatchFailure)
        swap_thread_error(dispatchError);
    return failed ? OVPHYSX_API_ERROR : OVPHYSX_API_SUCCESS;
}

// Runs one simulation step to completion under the caller's lane (see ovphysxSDK.hpp).
// Non-static because ovphysxTensorBinding.cpp uses it for warmup.
ovphysx_api_status_t omni_sdk_physx_simulate_instance(InstanceData& instance, ovphysx_instance_handle_t handle,
                                                      float elapsedTime, float currentTime, bool& dispatched)
{
    const ovphysx_api_status_t status = dispatch_simulation(instance, handle, elapsedTime, currentTime, dispatched);
    if (!dispatched)
        return status;
    return fetch_simulation(instance, status != OVPHYSX_API_SUCCESS);
}

struct LiveAttachment
{
    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    omni::physics::tensors::AttachHandle attach = omni::physics::tensors::kNoAttach;
};

// Each public instance owns at most one runtime attachment. Several may be
// live at once. Callers that need "is anything attached" scan the map;
// there is no process-wide owner handle.
static std::vector<LiveAttachment> collect_live_attachments()
{
    std::vector<LiveAttachment> live;
    std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
    live.reserve(g_instances.size());
    for (const auto& entry : g_instances)
    {
        if (!entry.second || entry.second->attachHandle == omni::physics::tensors::kNoAttach)
            continue;
        live.push_back(LiveAttachment{entry.first, entry.second->attachHandle});
    }
    return live;
}

// True when any instance other than `excluded` holds a runtime attachment.
// Pass OVPHYSX_INVALID_HANDLE to ask whether any instance is attached.
static bool other_instance_attached(ovphysx_instance_handle_t excluded)
{
    std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
    for (const auto& entry : g_instances)
    {
        if (entry.first == excluded || !entry.second)
            continue;
        if (entry.second->attachHandle != omni::physics::tensors::kNoAttach)
            return true;
    }
    return false;
}

static bool any_instance_attached()
{
    return other_instance_attached(OVPHYSX_INVALID_HANDLE);
}

namespace { // reopen anonymous namespace

    // ========================================================================
    // Typed config system: enum to Carbonite path lookup tables
    //
    // Paths shared with the scoped GlobalConfigSpec table below (see
    // getGlobalConfigSpec()) are named constants so the legacy and scoped
    // config paths cannot silently desync.
    // ========================================================================
    static constexpr const char* kPathDisableContactProcessing = "/physics/disableContactProcessing";
    // Public "custom geometry" is the inverse of these approximate-mesh settings.
    // Approximate true selects a convex mesh and ignores the convex-core margin.
    static constexpr const char* kPathCollisionApproximateCones = "/physics/collisionApproximateCones";
    static constexpr const char* kPathCollisionApproximateCylinders = "/physics/collisionApproximateCylinders";
    static constexpr const char* kPathNumThreads = omni::physx::kSettingNumThreads;
    static constexpr const char* kPathSceneMultiGPUMode = "/physics/sceneMultiGPUMode";
    static constexpr const char* kPathOvstageReadPoolMaxMB = "/physics/ovstageReadPoolMaxMB";
    static constexpr const char* kPathCookedColliderCacheDirectory = "/UJITSO/datastore/localCachePath";

    static const char* s_boolKeyPaths[] = {
        kPathDisableContactProcessing,
        kPathCollisionApproximateCones,
        kPathCollisionApproximateCylinders,
        nullptr, // Retired OVD startup output.
        nullptr, // Retired OVD capability alias.
    };
    static_assert(std::size(s_boolKeyPaths) == OVPHYSX_CONFIG_BOOL_COUNT, "s_boolKeyPaths out of sync with enum");

    static const char* s_int32KeyPaths[] = {
        kPathNumThreads,
        kPathSceneMultiGPUMode,
        nullptr, // Retired OVD startup TCP port.
        nullptr, // Retired OVD startup TCP timeout.
        kPathOvstageReadPoolMaxMB,
        ovphysx::profiling::kSettingProfilerBackend,
    };
    static_assert(std::size(s_int32KeyPaths) == OVPHYSX_CONFIG_INT32_COUNT, "s_int32KeyPaths out of sync with enum");

    static const char* s_floatKeyPaths[] = {
        nullptr,
    };
    static_assert(OVPHYSX_CONFIG_FLOAT_COUNT == 0, "Update s_floatKeyPaths when float keys are added");

    static const char* s_stringKeyPaths[] = {
        nullptr, // Retired OVD startup directory.
        kPathCookedColliderCacheDirectory,
        nullptr, // Retired OVD startup transport.
        nullptr, // Retired OVD startup address.
        ovphysx::profiling::kSettingProfilerOutputPath,
    };
    static_assert(std::size(s_stringKeyPaths) == OVPHYSX_CONFIG_STRING_COUNT, "s_stringKeyPaths out of sync with enum");

    static bool configStringHasEmbeddedNull(const ovphysx_string_t& value);

    struct OwnedConfigValue
    {
        ovphysx_config_value_type_t type = OVPHYSX_CONFIG_VALUE_BOOL;
        bool boolValue = false;
        int64_t int64Value = 0;
        double doubleValue = 0.0;
        std::string stringValue;
    };

    struct GlobalConfigSpec
    {
        ovphysx_config_value_type_t type;
        const char* path;
        bool startupOnly;
    };

    std::mutex g_scopedConfigMutex;
    std::unordered_map<uint32_t, OwnedConfigValue> g_globalConfigValues;

    static bool getGlobalConfigSpec(ovphysx_global_config_key_t key, GlobalConfigSpec& out)
    {
        switch (key)
        {
        case OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING:
            out = {OVPHYSX_CONFIG_VALUE_BOOL, kPathDisableContactProcessing, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY:
            out = {OVPHYSX_CONFIG_VALUE_BOOL, kPathCollisionApproximateCones, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY:
            out = {OVPHYSX_CONFIG_VALUE_BOOL, kPathCollisionApproximateCylinders, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_NUM_THREADS:
            out = {OVPHYSX_CONFIG_VALUE_INT64, kPathNumThreads, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE:
            out = {OVPHYSX_CONFIG_VALUE_INT64, kPathSceneMultiGPUMode, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB:
            out = {OVPHYSX_CONFIG_VALUE_INT64, kPathOvstageReadPoolMaxMB, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND:
            out = {OVPHYSX_CONFIG_VALUE_INT64, ovphysx::profiling::kSettingProfilerBackend, true};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY:
            out = {OVPHYSX_CONFIG_VALUE_STRING, kPathCookedColliderCacheDirectory, true};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH:
            out = {OVPHYSX_CONFIG_VALUE_STRING, ovphysx::profiling::kSettingProfilerOutputPath, true};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_CPU_ONLY:
            out = {OVPHYSX_CONFIG_VALUE_BOOL, nullptr, true};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL:
            out = {OVPHYSX_CONFIG_VALUE_INT64, nullptr, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT:
            out = {OVPHYSX_CONFIG_VALUE_BOOL, nullptr, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE:
            out = {OVPHYSX_CONFIG_VALUE_BOOL, omni::physx::kOmniPvdRecordingCapable, true};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION:
            out = {OVPHYSX_CONFIG_VALUE_BOOL, nullptr, false};
            return true;
        case OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE:
            // No Carbonite path: the retained value is the only storage, read by attach.
            out = {OVPHYSX_CONFIG_VALUE_INT64, nullptr, false};
            return true;
        default:
            return false;
        }
    }

    static ovphysx_api_status_t validateGlobalConfigValue(
        ovphysx_global_config_key_t key,
        const GlobalConfigSpec& spec,
        const ovphysx_config_value_t& value,
        const char*& outError)
    {
        if (value.type != spec.type)
        {
            outError = "Configuration value type does not match the key";
            return OVPHYSX_API_INVALID_ARGUMENT;
        }

        if (value.type == OVPHYSX_CONFIG_VALUE_STRING)
        {
            if ((!value.value.string_value.ptr && value.value.string_value.length != 0) ||
                configStringHasEmbeddedNull(value.value.string_value))
            {
                outError = "Configuration string is invalid or contains an embedded NUL";
                return OVPHYSX_API_INVALID_ARGUMENT;
            }
        }

        if (value.type != OVPHYSX_CONFIG_VALUE_INT64)
            return OVPHYSX_API_SUCCESS;

        const int64_t number = value.value.int64_value;
        if (number < std::numeric_limits<int32_t>::min() ||
            number > std::numeric_limits<int32_t>::max())
        {
            outError = "Configuration integer does not fit backend int32 storage";
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
        if (key == OVPHYSX_GLOBAL_CONFIG_NUM_THREADS && (number < 0 || number >= 256))
        {
            outError = "Worker thread count must be in 0..255";
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
        if (key == OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE &&
            (number < kMultiGPU_Disabled || number > kMultiGPU_SkipFirst))
        {
            outError = "Scene multi-GPU mode must be 0, 1, or 2";
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
        if (key == OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND &&
            (number < OVPHYSX_PROFILER_BACKEND_NONE || number > OVPHYSX_PROFILER_BACKEND_TRACE))
        {
            outError = "Profiler backend is out of range";
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
        if (key == OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL &&
            (number < OVPHYSX_LOG_DEFAULT || number > OVPHYSX_LOG_NONE))
        {
            outError = "Log level is out of range";
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
        if (key == OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE &&
            (number < OVPHYSX_COMPILED_STAGE_CACHE_AUTOMATIC ||
             number > OVPHYSX_COMPILED_STAGE_CACHE_READ_WRITE))
        {
            outError = "Compiled stage cache mode is out of range";
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
        return OVPHYSX_API_SUCCESS;
    }

    static OwnedConfigValue ownConfigValue(const ovphysx_config_value_t& value)
    {
        OwnedConfigValue owned;
        owned.type = value.type;
        switch (value.type)
        {
        case OVPHYSX_CONFIG_VALUE_BOOL:
            owned.boolValue = value.value.bool_value;
            break;
        case OVPHYSX_CONFIG_VALUE_INT64:
            owned.int64Value = value.value.int64_value;
            break;
        case OVPHYSX_CONFIG_VALUE_DOUBLE:
            owned.doubleValue = value.value.double_value;
            break;
        case OVPHYSX_CONFIG_VALUE_STRING:
            owned.stringValue.assign(
                value.value.string_value.ptr ? value.value.string_value.ptr : "",
                value.value.string_value.length);
            break;
        }
        return owned;
    }

    static bool pathIsApproximateCollision(const char* path)
    {
        return path &&
               (std::strcmp(path, kPathCollisionApproximateCones) == 0 ||
                std::strcmp(path, kPathCollisionApproximateCylinders) == 0);
    }

    static void applyOwnedConfigValue(
        carb::settings::ISettings* settings,
        const GlobalConfigSpec& spec,
        const OwnedConfigValue& value)
    {
        if (!settings || !spec.path)
            return;
        switch (value.type)
        {
        case OVPHYSX_CONFIG_VALUE_BOOL:
            settings->setBool(
                spec.path,
                pathIsApproximateCollision(spec.path) ? !value.boolValue : value.boolValue);
            break;
        case OVPHYSX_CONFIG_VALUE_INT64:
            settings->setInt(spec.path, static_cast<int32_t>(value.int64Value));
            break;
        case OVPHYSX_CONFIG_VALUE_DOUBLE:
            settings->setFloat(spec.path, static_cast<float>(value.doubleValue));
            break;
        case OVPHYSX_CONFIG_VALUE_STRING:
            settings->setString(spec.path, value.stringValue.c_str());
            break;
        }
    }

    static void applyPendingGlobalConfig(carb::settings::ISettings* settings)
    {
        struct PendingGlobalConfig
        {
            GlobalConfigSpec spec;
            OwnedConfigValue value;
        };

        // Snapshot under the lock, then write settings after releasing it.
        // Carbonite notifies subscribers on the writer thread, and a subscriber
        // that calls back into ovphysx_set_global_config must be able to take
        // this same non-recursive mutex.
        std::vector<PendingGlobalConfig> pending;
        {
            std::lock_guard<std::mutex> lock(g_scopedConfigMutex);
            pending.reserve(g_globalConfigValues.size());
            for (const std::pair<const uint32_t, OwnedConfigValue>& item : g_globalConfigValues)
            {
                GlobalConfigSpec spec;
                const ovphysx_global_config_key_t key =
                    static_cast<ovphysx_global_config_key_t>(item.first);
                if (!getGlobalConfigSpec(key, spec))
                    continue;
                PendingGlobalConfig entry;
                entry.spec = spec;
                entry.value = item.second;
                pending.push_back(std::move(entry));
            }
        }
        for (const PendingGlobalConfig& entry : pending)
            applyOwnedConfigValue(settings, entry.spec, entry.value);
    }

    // Forward declaration (defined below).
    static void applySettingValue(carb::settings::ISettings* settings, const char* key, const char* value);

    static bool configStringHasEmbeddedNull(const ovphysx_string_t& value)
    {
        return value.ptr && std::memchr(value.ptr, '\0', value.length) != nullptr;
    }

    enum class WorkerPoolEntryKind
    {
        eNone,
        eTasking,
        eTbb,
    };

    template <size_t N>
    static bool configCarbonitePathEquals(const ovphysx_string_t& value, const char (&expected)[N])
    {
        if (!value.ptr)
            return false;

        const char* valuePtr = value.ptr;
        size_t valueLength = value.length;
        if (valueLength > 0 && valuePtr[0] == '/')
        {
            ++valuePtr;
            --valueLength;
        }

        constexpr size_t expectedLength = N - 2;
        return expected[0] == '/' && expected[N - 1] == '\0' && valueLength == expectedLength &&
               std::memcmp(valuePtr, expected + 1, expectedLength) == 0;
    }

    static WorkerPoolEntryKind getWorkerPoolEntryKind(const ovphysx_config_entry_t& entry)
    {
        if (entry.key_type != OVPHYSX_CONFIG_KEY_TYPE_CARBONITE || !entry.key.carbonite_key.ptr)
            return WorkerPoolEntryKind::eNone;

        if (configCarbonitePathEquals(
                entry.key.carbonite_key, "/plugins/carb.tasking.plugin/threadCount"))
            return WorkerPoolEntryKind::eTasking;
        if (configCarbonitePathEquals(
                entry.key.carbonite_key, "/plugins/omni.tbb.globalcontrol/maxThreadCount"))
            return WorkerPoolEntryKind::eTbb;
        return WorkerPoolEntryKind::eNone;
    }

    static bool isWorkerPoolStartupEntry(const ovphysx_config_entry_t& entry)
    {
        return getWorkerPoolEntryKind(entry) != WorkerPoolEntryKind::eNone;
    }

    static ovphysx_api_status_t validateWorkerPoolStartupEntry(const ovphysx_config_entry_t& entry)
    {
        const WorkerPoolEntryKind kind = getWorkerPoolEntryKind(entry);
        if (kind == WorkerPoolEntryKind::eNone)
            return OVPHYSX_API_SUCCESS;
        if (!entry.value.string_value.ptr || configStringHasEmbeddedNull(entry.value.string_value))
            return OVPHYSX_API_INVALID_ARGUMENT;

        const char* begin = entry.value.string_value.ptr;
        const char* end = begin + entry.value.string_value.length;
        int32_t count = 0;
        const std::from_chars_result parsed = std::from_chars(begin, end, count, 10);
        if (parsed.ec != std::errc{} || parsed.ptr != end)
            return OVPHYSX_API_INVALID_ARGUMENT;

        // carb.tasking uses zero as its machine-sized default. TBB distinguishes
        // an absent setting from explicit zero, which global_control rejects.
        if (count < 0 || (kind == WorkerPoolEntryKind::eTbb && count == 0))
            return OVPHYSX_API_INVALID_ARGUMENT;
        return OVPHYSX_API_SUCCESS;
    }

    static bool isTaskingStarted()
    {
        carb::Framework* framework = carb::getFramework();
        return framework &&
               framework->tryAcquireExistingInterface<carb::tasking::ITasking>() != nullptr;
    }

    static bool globalKeyForSettingsPath(const char* path, ovphysx_global_config_key_t& outKey)
    {
        if (!path)
            return false;
        const ovphysx_global_config_key_t keys[] = {
            OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING,
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY,
            OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY,
            OVPHYSX_GLOBAL_CONFIG_NUM_THREADS,
            OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE,
            OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB,
            OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND,
            OVPHYSX_GLOBAL_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY,
            OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH,
        };
        // Carbonite treats one leading slash as optional, so persistent/physics/numThreads
        // and /persistent/physics/numThreads are the same item. Compare both without it.
        const char* normalized = path;
        if (normalized[0] == '/')
            ++normalized;
        if (normalized[0] == '\0')
            return false;
        for (ovphysx_global_config_key_t key : keys)
        {
            GlobalConfigSpec spec;
            if (getGlobalConfigSpec(key, spec) && spec.path && spec.path[0] == '/' &&
                std::strcmp(normalized, spec.path + 1) == 0)
            {
                outKey = key;
                return true;
            }
        }
        return false;
    }

    static bool legacyEntryMapsToGlobalKey(
        const ovphysx_config_entry_t& entry,
        ovphysx_global_config_key_t& outKey)
    {
        switch (entry.key_type)
        {
        case OVPHYSX_CONFIG_KEY_TYPE_BOOL:
            switch (entry.key.bool_key)
            {
            case OVPHYSX_CONFIG_DISABLE_CONTACT_PROCESSING:
                outKey = OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING;
                return true;
            case OVPHYSX_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY:
                outKey = OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY;
                return true;
            case OVPHYSX_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY:
                outKey = OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY;
                return true;
            default:
                return false;
            }
        case OVPHYSX_CONFIG_KEY_TYPE_INT32:
            switch (entry.key.int32_key)
            {
            case OVPHYSX_CONFIG_NUM_THREADS:
                outKey = OVPHYSX_GLOBAL_CONFIG_NUM_THREADS;
                return true;
            case OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE:
                outKey = OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE;
                return true;
            case OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB:
                outKey = OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB;
                return true;
            case OVPHYSX_CONFIG_PROFILER_BACKEND:
                outKey = OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND;
                return true;
            default:
                return false;
            }
        case OVPHYSX_CONFIG_KEY_TYPE_STRING:
            switch (entry.key.string_key)
            {
            case OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY:
                outKey = OVPHYSX_GLOBAL_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY;
                return true;
            case OVPHYSX_CONFIG_PROFILER_OUTPUT_PATH:
                outKey = OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH;
                return true;
            default:
                return false;
            }
        case OVPHYSX_CONFIG_KEY_TYPE_CARBONITE:
        {
            if (!entry.key.carbonite_key.ptr)
                return false;
            std::string path(entry.key.carbonite_key.ptr, entry.key.carbonite_key.length);
            return globalKeyForSettingsPath(path.c_str(), outKey);
        }
        default:
            return false;
        }
    }

    static void retainGlobalValueFromSettings(
        carb::settings::ISettings* settings,
        ovphysx_global_config_key_t key)
    {
        GlobalConfigSpec spec;
        if (!settings || !getGlobalConfigSpec(key, spec) || !spec.path)
            return;
        if (settings->getItemType(spec.path) == carb::dictionary::ItemType::eCount)
            return;

        OwnedConfigValue owned;
        owned.type = spec.type;
        switch (spec.type)
        {
        case OVPHYSX_CONFIG_VALUE_BOOL:
        {
            const bool stored = settings->getAsBool(spec.path);
            owned.boolValue = pathIsApproximateCollision(spec.path) ? !stored : stored;
            break;
        }
        case OVPHYSX_CONFIG_VALUE_INT64:
            owned.int64Value = settings->getAsInt(spec.path);
            break;
        case OVPHYSX_CONFIG_VALUE_STRING:
        {
            const char* text = settings->getStringBuffer(spec.path);
            if (!text)
                return;
            owned.stringValue = text;
            break;
        }
        default:
            return;
        }

        std::lock_guard<std::mutex> lock(g_scopedConfigMutex);
        g_globalConfigValues[static_cast<uint32_t>(key)] = std::move(owned);
    }

    static bool globalKeyRequiresDetachedRuntime(ovphysx_global_config_key_t key)
    {
        return key == OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY ||
               key == OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY ||
               key == OVPHYSX_GLOBAL_CONFIG_NUM_THREADS ||
               key == OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE;
    }

    static bool carboniteBoolToken(const std::string& text, bool& out)
    {
        if (text == "true" || text == "True" || text == "TRUE")
        {
            out = true;
            return true;
        }
        if (text == "false" || text == "False" || text == "FALSE")
        {
            out = false;
            return true;
        }
        return false;
    }

    // Public value for an overlapping legacy entry. Cone and cylinder Carbonite
    // strings are the approximate-mesh setting, so the public custom-geometry
    // value is the inverse. Typed entries already carry the public value.
    static bool legacyEntryToScopedValue(
        const ovphysx_config_entry_t& entry,
        ovphysx_global_config_key_t key,
        const GlobalConfigSpec& spec,
        ovphysx_config_value_t& out,
        const char*& outError)
    {
        out = {};
        out.type = spec.type;
        switch (entry.key_type)
        {
        case OVPHYSX_CONFIG_KEY_TYPE_BOOL:
            if (spec.type != OVPHYSX_CONFIG_VALUE_BOOL)
            {
                outError = "Configuration value type does not match the key";
                return false;
            }
            out.value.bool_value = entry.value.bool_value;
            return true;
        case OVPHYSX_CONFIG_KEY_TYPE_INT32:
            if (spec.type != OVPHYSX_CONFIG_VALUE_INT64)
            {
                outError = "Configuration value type does not match the key";
                return false;
            }
            out.value.int64_value = entry.value.int32_value;
            return true;
        case OVPHYSX_CONFIG_KEY_TYPE_STRING:
            if (spec.type != OVPHYSX_CONFIG_VALUE_STRING)
            {
                outError = "Configuration value type does not match the key";
                return false;
            }
            out.value.string_value = entry.value.string_value;
            return true;
        case OVPHYSX_CONFIG_KEY_TYPE_CARBONITE:
        {
            if (!entry.value.string_value.ptr && entry.value.string_value.length != 0)
            {
                outError = "Configuration string is invalid or contains an embedded NUL";
                return false;
            }
            const std::string text(
                entry.value.string_value.ptr ? entry.value.string_value.ptr : "",
                entry.value.string_value.length);
            if (spec.type == OVPHYSX_CONFIG_VALUE_BOOL)
            {
                bool parsed = false;
                if (!carboniteBoolToken(text, parsed))
                {
                    outError = "Configuration value type does not match the key";
                    return false;
                }
                if (key == OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY ||
                    key == OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY)
                    parsed = !parsed;
                out.value.bool_value = parsed;
                return true;
            }
            if (spec.type == OVPHYSX_CONFIG_VALUE_INT64)
            {
                int64_t parsed = 0;
                const std::from_chars_result result =
                    std::from_chars(text.data(), text.data() + text.size(), parsed, 10);
                if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
                {
                    outError = "Configuration value type does not match the key";
                    return false;
                }
                out.value.int64_value = parsed;
                return true;
            }
            if (spec.type == OVPHYSX_CONFIG_VALUE_STRING)
            {
                out.value.string_value = entry.value.string_value;
                return true;
            }
            outError = "Configuration value type does not match the key";
            return false;
        }
        default:
            outError = "Configuration value type does not match the key";
            return false;
        }
    }

    // Keys that reach every live attachment or need a detached runtime act on every
    // instance: their writes take every instance's lane first (AllInstanceLanes).
    static bool globalKeyNeedsEveryLane(ovphysx_global_config_key_t key)
    {
        return key == OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING || globalKeyRequiresDetachedRuntime(key);
    }

    static bool entryNeedsEveryLane(const ovphysx_config_entry_t& entry)
    {
        ovphysx_global_config_key_t key;
        return legacyEntryMapsToGlobalKey(entry, key) && globalKeyNeedsEveryLane(key);
    }

    // @implements REQ-CAPI-CONFIG-001
    // @covers AC-4 AC-7
    // Called with every instance's lane held, so no instance runs work and no dispatched
    // step is pending. The probe catches what no lane covers, such as companion services.
    static ovphysx_api_status_t updateLiveContactProcessing(bool disabled)
    {
        const std::vector<LiveAttachment> live = collect_live_attachments();
        if (live.empty())
            return OVPHYSX_API_SUCCESS;
        if (omni::physx::runtime::isRuntimeExecuting())
            return OVPHYSX_API_INVALID_STATE;
        for (const LiveAttachment& attachment : live)
        {
            if (!omni::physx::runtime::setAttachmentContactProcessingDisabled(attachment.attach, disabled))
                return OVPHYSX_API_ERROR;
        }
        return OVPHYSX_API_SUCCESS;
    }

    // Overlapping legacy keys share the scoped setter's range and lifecycle
    // checks. Create-time entries may still apply startup-only values; that is
    // the moment those values take effect. Tasking/TBB keys have separate checks.
    static ovphysx_api_status_t rejectOverlappingLegacyWrite(
        const ovphysx_config_entry_t& entry,
        bool duringInstanceCreate,
        const char*& outError)
    {
        outError = nullptr;
        ovphysx_global_config_key_t key;
        if (!legacyEntryMapsToGlobalKey(entry, key))
            return OVPHYSX_API_SUCCESS;
        GlobalConfigSpec spec;
        if (!getGlobalConfigSpec(key, spec))
            return OVPHYSX_API_SUCCESS;

        ovphysx_config_value_t value = {};
        if (!legacyEntryToScopedValue(entry, key, spec, value, outError))
            return OVPHYSX_API_INVALID_ARGUMENT;

        const ovphysx_api_status_t validation =
            validateGlobalConfigValue(key, spec, value, outError);
        if (validation != OVPHYSX_API_SUCCESS)
            return validation;

        if (!duringInstanceCreate && spec.startupOnly &&
            g_runtimeConfigStarted.load(std::memory_order_acquire))
        {
            outError =
                "Startup-only process-global configuration cannot change after instance creation";
            return OVPHYSX_API_INVALID_STATE;
        }
        if (globalKeyRequiresDetachedRuntime(key) && any_instance_attached())
        {
            outError = "This process-global configuration cannot change while a stage is attached";
            return OVPHYSX_API_INVALID_STATE;
        }
        return OVPHYSX_API_SUCCESS;
    }

    static ovphysx_api_status_t applyConfigEntry(carb::settings::ISettings* settings,
                                                  const ovphysx_config_entry_t& entry,
                                                  bool duringInstanceCreate,
                                                  const char*& outError)
    {
        outError = nullptr;
        if (!settings)
        {
            outError = "Settings interface not available";
            return OVPHYSX_API_ERROR;
        }
        const char* overlapError = nullptr;
        const ovphysx_api_status_t overlapStatus =
            rejectOverlappingLegacyWrite(entry, duringInstanceCreate, overlapError);
        if (overlapStatus != OVPHYSX_API_SUCCESS)
        {
            outError = overlapError;
            return overlapStatus;
        }
        ovphysx_global_config_key_t contactProcessingKey;
        if (legacyEntryMapsToGlobalKey(entry, contactProcessingKey) &&
            contactProcessingKey == OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING)
        {
            GlobalConfigSpec spec;
            getGlobalConfigSpec(contactProcessingKey, spec);
            ovphysx_config_value_t value{};
            if (!legacyEntryToScopedValue(entry, contactProcessingKey, spec, value, outError))
                return OVPHYSX_API_INVALID_ARGUMENT;
            const ovphysx_api_status_t status = updateLiveContactProcessing(value.value.bool_value);
            if (status != OVPHYSX_API_SUCCESS)
            {
                outError = "Contact-processing configuration could not reach the attached stage";
                return status;
            }
        }
        const auto finish = [&](ovphysx_api_status_t status) -> ovphysx_api_status_t
        {
            if (status == OVPHYSX_API_SUCCESS)
            {
                ovphysx_global_config_key_t key;
                if (legacyEntryMapsToGlobalKey(entry, key))
                    retainGlobalValueFromSettings(settings, key);
            }
            return status;
        };
        switch (entry.key_type)
        {
        case OVPHYSX_CONFIG_KEY_TYPE_BOOL:
            if (entry.key.bool_key < 0 || entry.key.bool_key >= OVPHYSX_CONFIG_BOOL_COUNT ||
                !s_boolKeyPaths[entry.key.bool_key])
                return OVPHYSX_API_INVALID_ARGUMENT;
            {
                const bool stored =
                    (entry.key.bool_key == OVPHYSX_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY ||
                     entry.key.bool_key == OVPHYSX_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY)
                        ? !entry.value.bool_value
                        : entry.value.bool_value;
                settings->setBool(s_boolKeyPaths[entry.key.bool_key], stored);
                CARB_LOG_INFO(
                    "[Config] Set bool %s = %s",
                    s_boolKeyPaths[entry.key.bool_key],
                    stored ? "true" : "false");
            }
            return finish(OVPHYSX_API_SUCCESS);
        case OVPHYSX_CONFIG_KEY_TYPE_INT32:
            if (entry.key.int32_key < 0 || entry.key.int32_key >= OVPHYSX_CONFIG_INT32_COUNT ||
                !s_int32KeyPaths[entry.key.int32_key])
                return OVPHYSX_API_INVALID_ARGUMENT;
            settings->setInt(s_int32KeyPaths[entry.key.int32_key], entry.value.int32_value);
            CARB_LOG_INFO("[Config] Set int %s = %d", s_int32KeyPaths[entry.key.int32_key], entry.value.int32_value);
            return finish(OVPHYSX_API_SUCCESS);
        case OVPHYSX_CONFIG_KEY_TYPE_FLOAT:
            if (entry.key.float_key < 0 || entry.key.float_key >= OVPHYSX_CONFIG_FLOAT_COUNT)
                return OVPHYSX_API_INVALID_ARGUMENT;
            settings->setFloat(s_floatKeyPaths[entry.key.float_key], entry.value.float_value);
            CARB_LOG_INFO("[Config] Set float %s = %g", s_floatKeyPaths[entry.key.float_key], (double)entry.value.float_value);
            return finish(OVPHYSX_API_SUCCESS);
        case OVPHYSX_CONFIG_KEY_TYPE_STRING:
            if (entry.key.string_key < 0 || entry.key.string_key >= OVPHYSX_CONFIG_STRING_COUNT ||
                !s_stringKeyPaths[entry.key.string_key])
                return OVPHYSX_API_INVALID_ARGUMENT;
            {
                if ((!entry.value.string_value.ptr && entry.value.string_value.length != 0) ||
                    (entry.key.string_key == OVPHYSX_CONFIG_PROFILER_OUTPUT_PATH &&
                     configStringHasEmbeddedNull(entry.value.string_value)))
                    return OVPHYSX_API_INVALID_ARGUMENT;
                std::string val(
                    entry.value.string_value.ptr ? entry.value.string_value.ptr : "",
                    entry.value.string_value.length);
                settings->setString(s_stringKeyPaths[entry.key.string_key], val.c_str());
                CARB_LOG_INFO("[Config] Set string %s = %s", s_stringKeyPaths[entry.key.string_key], val.c_str());
            }
            return finish(OVPHYSX_API_SUCCESS);
        case OVPHYSX_CONFIG_KEY_TYPE_CARBONITE:
            if (!entry.key.carbonite_key.ptr || !entry.value.string_value.ptr)
                return OVPHYSX_API_INVALID_ARGUMENT;
            {
                if (configStringHasEmbeddedNull(entry.key.carbonite_key) ||
                    configStringHasEmbeddedNull(entry.value.string_value))
                    return OVPHYSX_API_INVALID_ARGUMENT;
                std::string key(entry.key.carbonite_key.ptr, entry.key.carbonite_key.length);
                if (key == "/physics/cudaDevice") {
                    CARB_LOG_ERROR("[Config] Cannot set '/physics/cudaDevice' via carbonite config entry. Use active_cuda_gpus on create_args instead.");
                    return OVPHYSX_API_INVALID_ARGUMENT;
                }
                for (int i = 0; i < OVPHYSX_CONFIG_BOOL_COUNT; ++i)
                    if (s_boolKeyPaths[i] && key == s_boolKeyPaths[i]) { CARB_LOG_WARN("[Config] Carbonite key '%s' overlaps typed bool key %d; prefer the typed API.", key.c_str(), i); break; }
                for (int i = 0; i < OVPHYSX_CONFIG_INT32_COUNT; ++i)
                    if (s_int32KeyPaths[i] && key == s_int32KeyPaths[i]) { CARB_LOG_WARN("[Config] Carbonite key '%s' overlaps typed int32 key %d; prefer the typed API.", key.c_str(), i); break; }
                std::string val(entry.value.string_value.ptr, entry.value.string_value.length);
                applySettingValue(settings, key.c_str(), val.c_str());
            }
            return finish(OVPHYSX_API_SUCCESS);
        default:
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
    }

    static constexpr const char* kRetiredOvdConfigError =
        "Legacy OVD configuration was removed; set OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE "
        "before creating the first instance, then use ovphysx_ovd_start_recording";

    static bool isRetiredOvdConfigEntry(const ovphysx_config_entry_t& entry)
    {
        // These retired ABI slots must not acquire a new meaning.
        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_BOOL)
            return entry.key.bool_key == 3 || entry.key.bool_key == 4;

        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_INT32)
            return entry.key.int32_key == 2 || entry.key.int32_key == 3;

        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_STRING)
            return entry.key.string_key == 0 || entry.key.string_key == 2 || entry.key.string_key == 3;

        if (entry.key_type != OVPHYSX_CONFIG_KEY_TYPE_CARBONITE || !entry.key.carbonite_key.ptr)
            return false;

        // Retired paths survive only as migration errors, not runtime settings.
        return configCarbonitePathEquals(entry.key.carbonite_key, "/physics/omniPvdOutputEnabled") ||
               configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdRecordingCapable) ||
               configCarbonitePathEquals(entry.key.carbonite_key, "/persistent/physics/omniPvdOvdRecordingDirectory") ||
               configCarbonitePathEquals(entry.key.carbonite_key, "/physics/omniPvdTransport") ||
               configCarbonitePathEquals(entry.key.carbonite_key, "/physics/omniPvdTcpAddress") ||
               configCarbonitePathEquals(entry.key.carbonite_key, "/physics/omniPvdTcpPort") ||
               configCarbonitePathEquals(entry.key.carbonite_key, "/physics/omniPvdTcpTimeoutMs");
    }

    static ovphysx_api_status_t applyCreateConfigEntries(carb::settings::ISettings* settings,
                                                          const ovphysx_config_entry_t* entries,
                                                          uint32_t entryCount,
                                                          const char*& outError)
    {
        outError = nullptr;
        for (uint32_t i = 0; i < entryCount; ++i)
        {
            const char* entryError = nullptr;
            const ovphysx_api_status_t status =
                applyConfigEntry(settings, entries[i], true, entryError);
            if (status != OVPHYSX_API_SUCCESS)
            {
                outError = entryError;
                return status;
            }
        }
        return OVPHYSX_API_SUCCESS;
    }

    // ovphysx_set_global_config_entry() calls made before Carbonite starts. The
    // next create applies them once, before its own config entries, then drops
    // them. Guarded by g_createInstanceMutex.
    struct QueuedConfigEntry
    {
        ovphysx_config_entry_t entry{};
        std::string key;
        std::string value;
    };
    std::vector<QueuedConfigEntry> g_queuedConfigEntries;

    static const char* getConfigEntryPath(const ovphysx_config_entry_t& entry);

    static ovphysx_api_status_t queueConfigEntry(const ovphysx_config_entry_t& entry, const char*& outError)
    {
        outError = nullptr;
        if (entry.key_type != OVPHYSX_CONFIG_KEY_TYPE_CARBONITE && !getConfigEntryPath(entry))
            return OVPHYSX_API_INVALID_ARGUMENT;
        const ovphysx_api_status_t overlapStatus = rejectOverlappingLegacyWrite(entry, false, outError);
        if (overlapStatus != OVPHYSX_API_SUCCESS)
            return overlapStatus;
        QueuedConfigEntry queued;
        queued.entry = entry;
        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_CARBONITE)
        {
            if (!entry.key.carbonite_key.ptr || !entry.value.string_value.ptr ||
                configStringHasEmbeddedNull(entry.key.carbonite_key) ||
                configStringHasEmbeddedNull(entry.value.string_value))
                return OVPHYSX_API_INVALID_ARGUMENT;
            if (configCarbonitePathEquals(entry.key.carbonite_key, "/physics/cudaDevice"))
            {
                outError = "Cannot set /physics/cudaDevice via config entries. Use active_cuda_gpus instead.";
                return OVPHYSX_API_INVALID_ARGUMENT;
            }
            queued.key.assign(entry.key.carbonite_key.ptr, entry.key.carbonite_key.length);
            queued.value.assign(entry.value.string_value.ptr, entry.value.string_value.length);
        }
        else if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_STRING)
        {
            if (!entry.value.string_value.ptr && entry.value.string_value.length != 0)
                return OVPHYSX_API_INVALID_ARGUMENT;
            if (entry.value.string_value.ptr)
                queued.value.assign(entry.value.string_value.ptr, entry.value.string_value.length);
        }
        g_queuedConfigEntries.push_back(std::move(queued));
        return OVPHYSX_API_SUCCESS;
    }

    static ovphysx_api_status_t applyQueuedConfigEntries(carb::settings::ISettings* settings, const char*& outError)
    {
        outError = nullptr;
        std::vector<QueuedConfigEntry> queued;
        queued.swap(g_queuedConfigEntries);
        if (!queued.empty())
            CARB_LOG_INFO("[ovphysx] Applying %zu config entries set before Carbonite started", queued.size());
        for (const QueuedConfigEntry& item : queued)
        {
            ovphysx_config_entry_t entry = item.entry;
            if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_CARBONITE)
                entry.key.carbonite_key = {item.key.data(), item.key.size()};
            if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_CARBONITE || entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_STRING)
                entry.value.string_value = {item.value.data(), item.value.size()};
            const ovphysx_api_status_t status = applyConfigEntry(settings, entry, true, outError);
            if (status != OVPHYSX_API_SUCCESS)
                return status;
        }
        return OVPHYSX_API_SUCCESS;
    }

    static const char* getConfigEntryPath(const ovphysx_config_entry_t& entry)
    {
        switch (entry.key_type)
        {
        case OVPHYSX_CONFIG_KEY_TYPE_BOOL:
            return (entry.key.bool_key >= 0 && entry.key.bool_key < OVPHYSX_CONFIG_BOOL_COUNT) ? s_boolKeyPaths[entry.key.bool_key] : nullptr;
        case OVPHYSX_CONFIG_KEY_TYPE_INT32:
            return (entry.key.int32_key >= 0 && entry.key.int32_key < OVPHYSX_CONFIG_INT32_COUNT) ? s_int32KeyPaths[entry.key.int32_key] : nullptr;
        case OVPHYSX_CONFIG_KEY_TYPE_FLOAT:
            return (entry.key.float_key >= 0 && entry.key.float_key < OVPHYSX_CONFIG_FLOAT_COUNT) ? s_floatKeyPaths[entry.key.float_key] : nullptr;
        case OVPHYSX_CONFIG_KEY_TYPE_STRING:
            return (entry.key.string_key >= 0 && entry.key.string_key < OVPHYSX_CONFIG_STRING_COUNT) ? s_stringKeyPaths[entry.key.string_key] : nullptr;
        default:
            return nullptr;
        }
    }

    // Detects the value type (bool, int, float, string) and applies the setting.
    static void applySettingValue(carb::settings::ISettings* settings, const char* key, const char* value)
    {
        if (!settings || !key || !value) return;
        
        if (strcmp(value, "true") == 0 || strcmp(value, "True") == 0 || strcmp(value, "TRUE") == 0) {
            settings->setBool(key, true);
            CARB_LOG_INFO("[Settings] Set bool %s = true", key);
            return;
        }
        if (strcmp(value, "false") == 0 || strcmp(value, "False") == 0 || strcmp(value, "FALSE") == 0) {
            settings->setBool(key, false);
            CARB_LOG_INFO("[Settings] Set bool %s = false", key);
            return;
        }
        
        // Try integer via std::from_chars (C++17, avoids glibc __isoc23_strtol redirect).
        const char* valueEnd = value + std::strlen(value);
        {
            int32_t intVal = 0;
            auto [ptr, ec] = std::from_chars(value, valueEnd, intVal);
            if (ec == std::errc{} && ptr == valueEnd)
            {
                settings->setInt(key, intVal);
                CARB_LOG_INFO("[Settings] Set int %s = %d", key, intVal);
                return;
            }
            // Overflow is terminal only when the entire string is an integer. If
            // ptr stopped at '.', the value may be a float like "99999999999.5",
            // which the float parse below handles.
            if (ec == std::errc::result_out_of_range && ptr == valueEnd)
            {
                settings->setString(key, value);
                CARB_LOG_WARN("[Settings] int value out of int32 range for %s (stored as string)", key);
                return;
            }
        }

        // Try float via std::from_chars (C++17, avoids glibc __isoc23_strtod redirect).
        {
            double doubleVal = 0.0;
            auto [ptr, ec] = std::from_chars(value, valueEnd, doubleVal);
            if (ec == std::errc{} && ptr == valueEnd)
            {
                settings->setFloat(key, static_cast<float>(doubleVal));
                CARB_LOG_INFO("[Settings] Set float %s = %g", key, doubleVal);
                return;
            }
        }
        
        settings->setString(key, value);
        CARB_LOG_INFO("[Settings] Set string %s = %s", key, value);
    }
    
    // Formats a setting value as a string. Returns false when the setting does not exist.
    static bool getSettingValueAsString(carb::settings::ISettings* settings, const char* key,
                                        char* value_out, uint32_t value_out_size,
                                        size_t* out_required_size = nullptr)
    {
        if (!settings || !key || !value_out || value_out_size == 0) return false;

        // Writes the value into the caller's buffer and reports the full size,
        // including the null terminator, through out_required_size.
        auto writeAndReport = [&](const char* src) -> bool {
            size_t full_len = strlen(src);
            if (out_required_size)
                *out_required_size = full_len + 1;  // including null terminator
            strncpy(value_out, src, value_out_size - 1);
            value_out[value_out_size - 1] = '\0';
            return true;
        };

        const char* strVal = settings->getStringBuffer(key);
        if (strVal) {
            return writeAndReport(strVal);
        }

        // ISettings has no "exists" check, so use the dictionary item type to detect
        // what kind of value it is.
        auto itemType = settings->getItemType(key);

        if (itemType == carb::dictionary::ItemType::eBool) {
            bool boolVal = settings->getAsBool(key);
            char tmp[8];
            snprintf(tmp, sizeof(tmp), "%s", boolVal ? "true" : "false");
            return writeAndReport(tmp);
        }

        if (itemType == carb::dictionary::ItemType::eInt) {
            int32_t intVal = settings->getAsInt(key);
            char tmp[32];
            snprintf(tmp, sizeof(tmp), "%d", intVal);
            return writeAndReport(tmp);
        }

        if (itemType == carb::dictionary::ItemType::eFloat) {
            float floatVal = settings->getAsFloat(key);
            char tmp[64];
            snprintf(tmp, sizeof(tmp), "%g", floatVal);
            return writeAndReport(tmp);
        }

        if (itemType == carb::dictionary::ItemType::eString) {
            // Normally handled by the getStringBuffer call above.
            const char* str = settings->getStringBuffer(key);
            if (str) {
                return writeAndReport(str);
            }
        }

        // Setting doesn't exist or has unsupported type
        return false;
    }
    
} // namespace

namespace {
    // Recording mutates the one process-wide PhysX/OmniPVD runtime. The caller first takes
    // every instance's lane, which waits for queued and running jobs without collecting
    // task results, and only then companion transition admission, so no admission is held
    // while it waits. This checks the activity no lane covers (companion previews, runtime
    // work of an instance registered after the lanes were taken) and takes the transition
    // lock.
    static ovphysx_api_status_t enter_shared_runtime_safe_point(std::unique_lock<std::mutex>& transitionLock)
    {
        if (companionRuntime().hasPendingPreviews() || omni::physx::runtime::isRuntimeExecuting())
            return OVPHYSX_API_INVALID_STATE;
        transitionLock.lock();
        return OVPHYSX_API_SUCCESS;
    }

    // Must be called while holding g_gpuAttachMutex, after a shared-runtime
    // transition that may have stopped OmniPVD sampling. Preserve the public
    // owner when sampling is still active. Another handle may own the recording.
    static void reconcile_recording_owner_after_runtime_transition(
        omni::physx::IPhysxSimulation* physxSim)
    {
        bool recording = false;
        if (physxSim && physxSim->getOmniPvdRecordingStatus &&
            physxSim->getOmniPvdRecordingStatus(&recording) == omni::physx::RuntimeResult::eSuccess && !recording)
            setRecordingOwner(OVPHYSX_INVALID_HANDLE);
    }

    // Run only on the public submission thread, before it takes an instance lane.
    // A buffered receiver request uses the same all-instance barrier as start/stop.
    // Empty polls never wait for queued work, and no worker waits for its own lane.
    static void process_pending_omnipvd_capture_request(ovphysx_instance_handle_t handle)
    {
        const std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor || instance->executor->heldByCurrent() ||
            ovphysx::currentCaller().callback || legacyStreamHeldByCapture())
            return;
        omni::physx::IPhysxSimulation* simulation =
            instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
        if (!simulation || !simulation->hasPendingOmniPvdCaptureRequest || !simulation->processOmniPvdCaptureRequests)
            return;
        {
            std::unique_lock<std::mutex> transition(g_gpuAttachMutex, std::try_to_lock);
            if (!transition.owns_lock() || g_omniPvdRecordingOwner == OVPHYSX_INVALID_HANDLE)
                return;
            omni::physics::AttachHandle attach;
            {
                std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
                attach = instance->attachHandle;
            }
            if (!simulation->hasPendingOmniPvdCaptureRequest(attach))
                return;
        }
        ovphysx::AllInstanceLanes everyLane;
        if (everyLane.acquire() != OVPHYSX_API_SUCCESS)
            return;
        CompanionTransitionGuard companionTransition(companionRuntime());
        if (!companionTransition)
            return;
        std::unique_lock<std::mutex> transition(g_gpuAttachMutex, std::defer_lock);
        if (enter_shared_runtime_safe_point(transition) != OVPHYSX_API_SUCCESS ||
            g_omniPvdRecordingOwner == OVPHYSX_INVALID_HANDLE ||
            !simulation->hasPendingOmniPvdCaptureRequest(instance->attachHandle))
            return;
        if (simulation->processOmniPvdCaptureRequests(instance->attachHandle) == omni::physx::RuntimeResult::eSuccess)
            reconcile_recording_owner_after_runtime_transition(simulation);
    }

    // Recording is process-wide. The instance that started it can end it, and so
    // can detach of the last live attachment: that teardown is the recorded
    // scene's destruction, even when a stageless peer started the stream.
    // Detaching one attachment while another stays live does not stop a stream
    // that this instance did not start.
    static void finalize_recording_after_detach(
        omni::physx::IPhysxSimulation* physxSim, ovphysx_instance_handle_t handle)
    {
        // The runtime records only after ovphysx_ovd_start_recording() set an owner, which
        // is cleared only once the runtime reports no recording. Without an owner there is
        // nothing to finalize, and the runtime's status query, which briefly admits every
        // attachment, could refuse a peer instance's job running on its worker meanwhile.
        if (g_omniPvdRecordingOwner == OVPHYSX_INVALID_HANDLE)
            return;
        const bool lastLiveAttachment = !other_instance_attached(handle);
        if ((g_omniPvdRecordingOwner == handle || lastLiveAttachment) && recordingActive(physxSim))
        {
            if (physxSim->stopOmniPvdRecording() != omni::physx::OmniPvdRecordingResult::eSuccess)
                CARB_LOG_ERROR("Failed to finalize OmniPVD recording after detach");
        }
        reconcile_recording_owner_after_runtime_transition(physxSim);
    }

    static bool declaredGlobalDefault(ovphysx_global_config_key_t key, OwnedConfigValue& out)
    {
        GlobalConfigSpec spec;
        if (!getGlobalConfigSpec(key, spec))
            return false;
        if (spec.type == OVPHYSX_CONFIG_VALUE_STRING)
            return false;

        out = OwnedConfigValue();
        out.type = spec.type;
        switch (key)
        {
        case OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING:
        case OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE:
            out.boolValue = false;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_COLLISION_CONE_CUSTOM_GEOMETRY:
        case OVPHYSX_GLOBAL_CONFIG_COLLISION_CYLINDER_CUSTOM_GEOMETRY:
            // Backend approximate-mesh default is false, so custom geometry is on.
            out.boolValue = true;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_NUM_THREADS:
            out.int64Value = 8;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_SCENE_MULTI_GPU_MODE:
            out.int64Value = 0;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB:
            out.int64Value = 256;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND:
            out.int64Value = OVPHYSX_PROFILER_BACKEND_NONE;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_CPU_ONLY:
            out.boolValue = false;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL:
            out.int64Value = OVPHYSX_LOG_WARNING;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT:
        case OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION:
            out.boolValue = true;
            return true;
        case OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE:
            out.int64Value = OVPHYSX_COMPILED_STAGE_CACHE_AUTOMATIC;
            return true;
        default:
            return false;
        }
    }

    // Configured OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE value. The key has no
    // Carbonite path, so the retained value is its only storage.
    static int64_t configuredCompiledStageCacheMode()
    {
        std::lock_guard<std::mutex> lock(g_scopedConfigMutex);
        const std::unordered_map<uint32_t, OwnedConfigValue>::const_iterator found =
            g_globalConfigValues.find(static_cast<uint32_t>(OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE));
        return found != g_globalConfigValues.end() ? found->second.int64Value
                                                   : static_cast<int64_t>(OVPHYSX_COMPILED_STAGE_CACHE_AUTOMATIC);
    }

    static omni::physx::PhysxCompiledStageCacheMode resolveCompiledStageCacheMode(int64_t configured)
    {
        using Mode = omni::physx::PhysxCompiledStageCacheMode;
        switch (configured)
        {
        case OVPHYSX_COMPILED_STAGE_CACHE_DISABLED:
            return Mode::eDisabled;
        case OVPHYSX_COMPILED_STAGE_CACHE_READ_ONLY:
            return Mode::eReadOnly;
        case OVPHYSX_COMPILED_STAGE_CACHE_READ_WRITE:
            return Mode::eReadWrite;
        case OVPHYSX_COMPILED_STAGE_CACHE_AUTOMATIC:
        default:
            // Store only into a cooked-collider directory the application configured,
            // while UJITSO collision cooking is on. The process-private fallback is
            // deleted at shutdown, so every run would pay the store cost and reuse its
            // bundles only within this process.
            return ovphysx::usesConfiguredCookedColliderCache() ? Mode::eReadWrite : Mode::eDisabled;
        }
    }

    static const char* compiledStageCacheModeName(omni::physx::PhysxCompiledStageCacheMode mode)
    {
        using Mode = omni::physx::PhysxCompiledStageCacheMode;
        switch (mode)
        {
        case Mode::eDisabled:
            return "disabled";
        case Mode::eReadOnly:
            return "read-only";
        case Mode::eReadWrite:
            return "read-write";
        default:
            return "unknown";
        }
    }

    static const char* compiledStageCacheResultName(omni::physx::PhysxCompiledStageCacheResult result)
    {
        using Result = omni::physx::PhysxCompiledStageCacheResult;
        switch (result)
        {
        case Result::eDisabled:
            return "disabled";
        case Result::eBypassed:
            return "bypassed";
        case Result::eMiss:
            return "miss";
        case Result::eStored:
            return "stored";
        case Result::eHit:
            return "hit";
        case Result::eRestoreFailed:
            return "restore failed";
        case Result::eStoreFailed:
            return "store failed";
        case Result::eUnavailable:
            return "unavailable";
        default:
            return "unknown";
        }
    }

    // OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION as an instance created now reads it.
    static bool asyncExecutionEnabled()
    {
        OwnedConfigValue value;
        {
            std::lock_guard<std::mutex> lock(g_scopedConfigMutex);
            const std::unordered_map<uint32_t, OwnedConfigValue>::const_iterator found =
                g_globalConfigValues.find(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION);
            if (found != g_globalConfigValues.end())
                return found->second.boolValue;
        }
        declaredGlobalDefault(OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION, value);
        return value.boolValue;
    }

    static std::shared_ptr<ovphysx::InstanceExecutor> createInstanceExecutor()
    {
        if (!asyncExecutionEnabled())
            return ovphysx::InstanceExecutor::createWithoutWorker();
        carb::Framework* framework = carb::getFramework();
        carb::tasking::IThreadPool* threads =
            framework ? framework->tryAcquireExistingInterface<carb::tasking::IThreadPool>() : nullptr;
        carb::tasking::ITasking* tasking = g_executorTasking.load(std::memory_order_acquire);
        const uint32_t poolThreads = tasking ? tasking->getDesc().threadCount : 0;
        if (threads && poolThreads > 1)
            return ovphysx::InstanceExecutor::create(*threads);
        CARB_LOG_WARN("[ovphysx] The instance runs synchronously: asynchronous execution disabled %s (%u threads). "
                      "Set OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION to false to run synchronously without this warning.",
                      threads ? "for this pool size" : "without carb::tasking::IThreadPool", poolThreads);
        return ovphysx::InstanceExecutor::createWithoutWorker();
    }
}

// ---------------------------------------------------------------------------
// Instance lanes: callback refusal, one instance's lane, every instance's lane.
// ---------------------------------------------------------------------------

ovphysx::Caller ovphysx::currentCaller() noexcept
{
    // One carb.tasking query serves the lane identity and every callback check.
    carb::tasking::ITasking* tasking = g_executorTasking.load(std::memory_order_acquire);
    const uint64_t task = tasking ? static_cast<uint64_t>(tasking->getTaskContext()) : 0;
    Caller caller;
    caller.identity = InstanceExecutor::identityOf(task);
    if (ovphysx::isInLogCallback())
        caller.callback = "inside a log callback";
    else if (tasking ? omni::physx::runtime::isInRuntimeCallback(task) : omni::physx::runtime::isInRuntimeCallback())
        caller.callback = "inside a runtime callback";
    else if (ovphysx_object_change_callback_active(tasking, task))
        caller.callback = "inside an object-change callback";
    return caller;
}

const char* ovphysx::callbackContext() noexcept
{
    return currentCaller().callback;
}

ovphysx_api_status_t ovphysx::refuseBlockingReentry(const InstanceData* instance) noexcept
{
    return refuseBlockingReentry(instance, currentCaller());
}

ovphysx_api_status_t ovphysx::refuseBlockingReentry(const InstanceData* instance, const Caller& caller) noexcept
{
    const char* where = caller.callback;
    if (!where && instance && instance->executor && instance->executor->isWorkerIdentity(caller.identity))
        where = "on the instance's own worker thread";
    if (!where)
        return OVPHYSX_API_SUCCESS;
    char message[160];
    std::snprintf(message, sizeof(message),
                  "This call waits for instance work and cannot run %s; retry after the callback returns", where);
    return set_error(OVPHYSX_API_INVALID_STATE, message).status;
}

ovphysx_api_status_t ovphysx::InstanceLane::acquire(ovphysx_instance_handle_t handle)
{
    release();
    const Caller caller = currentCaller();
    if (caller.callback)
        return refuseBlockingReentry(nullptr, caller);
    m_instance = get_instance(handle);
    if (!m_instance || !m_instance->executor)
    {
        m_instance.reset();
        return OVPHYSX_API_NOT_FOUND;
    }
    if (m_instance->executor->isWorkerIdentity(caller.identity))
    {
        const ovphysx_api_status_t refused = refuseBlockingReentry(m_instance.get(), caller);
        m_instance.reset();
        return refused;
    }
    return take(*m_instance, caller.identity);
}

ovphysx_api_status_t ovphysx::InstanceLane::acquire(InstanceData& instance, const Caller& caller)
{
    release();
    if (!instance.executor)
        return OVPHYSX_API_NOT_FOUND;
    return take(instance, caller.identity);
}

ovphysx_api_status_t ovphysx::InstanceLane::take(InstanceData& instance, uint64_t identity)
{
    m_hold = instance.executor->acquireLane(identity);
    // Destroy stops the worker while it holds the lane and unregisters the instance
    // before releasing it, so a caller that took the lane after it finds it stopped; so
    // does every call after ovphysx_shutdown(). Like a job submission it is refused, and
    // runs no engine code.
    if (instance.executor->stopped())
    {
        release();
        return set_error(OVPHYSX_API_INVALID_STATE, "Instance worker is stopped; destroy the instance").status;
    }
    return OVPHYSX_API_SUCCESS;
}

ovphysx_api_status_t ovphysx::InstanceLane::preempt(ovphysx_instance_handle_t handle)
{
    release();
    m_instance = get_instance(handle);
    if (!m_instance || !m_instance->executor)
    {
        m_instance.reset();
        return OVPHYSX_API_NOT_FOUND;
    }
    if (refuseBlockingReentry(m_instance.get()) != OVPHYSX_API_SUCCESS)
    {
        m_instance.reset();
        return OVPHYSX_API_INVALID_STATE;
    }
    m_hold = m_instance->executor->preemptLane();
    return OVPHYSX_API_SUCCESS;
}

ovphysx_api_status_t ovphysx::AllInstanceLanes::acquire(ovphysx_instance_handle_t preempted)
{
    if (refuseBlockingReentry(nullptr) != OVPHYSX_API_SUCCESS)
        return OVPHYSX_API_INVALID_STATE;
    for (;;)
    {
        std::vector<std::pair<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>> missing;
        {
            std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
            for (const std::pair<const ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>& entry : g_instances)
            {
                if (entry.second && entry.second->executor && !holds(entry.first))
                    missing.emplace_back(entry.first, entry.second);
            }
        }
        if (missing.empty())
            return OVPHYSX_API_SUCCESS;
        std::sort(missing.begin(), missing.end(),
                  [](const auto& left, const auto& right) { return left.first < right.first; });
        // Instances register in handle order, so later registrations extend the
        // ascending sequence. Restart if one ever does not.
        if (!m_entries.empty() && missing.front().first < m_entries.back().handle)
        {
            release();
            continue;
        }
        for (const std::pair<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>& candidate : missing)
        {
            if (candidate.second->executor->isWorkerThread())
            {
                release();
                return refuseBlockingReentry(candidate.second.get());
            }
        }
        m_entries.reserve(m_entries.size() + missing.size());
        for (const std::pair<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>& candidate : missing)
        {
            Entry entry;
            entry.handle = candidate.first;
            entry.executor = candidate.second->executor;
            entry.hold = candidate.first == preempted
                             ? entry.executor->preemptLane()
                             : entry.executor->acquireLane();
            m_entries.push_back(std::move(entry));
        }
    }
}

bool ovphysx::AllInstanceLanes::holds(ovphysx_instance_handle_t handle) const noexcept
{
    const std::vector<Entry>::const_iterator found = std::lower_bound(
        m_entries.begin(), m_entries.end(), handle,
        [](const Entry& entry, ovphysx_instance_handle_t value) { return entry.handle < value; });
    return found != m_entries.end() && found->handle == handle;
}

bool ovphysx::AllInstanceLanes::coversRegistered() const
{
    std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
    for (const std::pair<const ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>& entry : g_instances)
    {
        if (entry.second && entry.second->executor && !holds(entry.first))
            return false;
    }
    return true;
}

void ovphysx::AllInstanceLanes::release() noexcept
{
    while (!m_entries.empty())
        m_entries.pop_back();
}

extern "C" {

ovphysx_api_status_t omni_sdk_physx_set_setting(
    ovphysx_instance_handle_t handle,
    const char* key,
    const char* value
)
{
    if (!key || !value) {
        return OVPHYSX_API_ERROR;
    }
    
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance) {
        return OVPHYSX_API_ERROR;
    }

    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) {
        CARB_LOG_ERROR("[ovphysx] Error: ISettings not available for set_setting");
        return OVPHYSX_API_ERROR;
    }
    
    applySettingValue(settings, key, value);
    return OVPHYSX_API_SUCCESS;
}

ovphysx_api_status_t omni_sdk_physx_get_setting(
    ovphysx_instance_handle_t handle,
    const char* key,
    char* value_out,
    uint32_t value_out_size
)
{
    if (!key || !value_out || value_out_size == 0) {
        return OVPHYSX_API_ERROR;
    }
    
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance) {
        return OVPHYSX_API_ERROR;
    }

    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) {
        CARB_LOG_ERROR("[ovphysx] Error: ISettings not available for get_setting");
        return OVPHYSX_API_ERROR;
    }
    
    if (getSettingValueAsString(settings, key, value_out, value_out_size)) {
        return OVPHYSX_API_SUCCESS;
    }

    return OVPHYSX_API_ERROR;
}

ovphysx_api_status_t omni_sdk_physx_unload_usd(ovphysx_instance_handle_t handle)
{
    const std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
    InstanceData* instance = instanceShared.get();
    if (!instance) {
        return OVPHYSX_API_ERROR;
    }

    if (instance->attachedStageId != 0) {
        try {
            int64_t stageId = instance->attachedStageId;
            const bool rawUnknownCaller = (g_threadUnloadCaller == UnloadCaller::kUnknown);
            UnloadCaller caller = g_threadUnloadCaller;
            if (caller == UnloadCaller::kUnknown)
            {
                caller = UnloadCaller::kAPI;
            }
            uint32_t unloadSeq = ++g_unloadSequenceCounter;
            auto threadId = std::this_thread::get_id();
            if (rawUnknownCaller)
            {
                logCallerSymbol(unloadSeq);
                logCallStackForUnload(unloadSeq);
            }

            StageLifecycleEntry* stageEntry = getStageLifecycleEntry(stageId);
            if (!stageEntry)
            {
                CARB_LOG_INFO("[UNLOAD #%u] (caller=%s) Stage %" PRId64 " not tracked - assuming detached, skipping",
                             unloadSeq, toString(caller), stageId);
                return OVPHYSX_API_SUCCESS;
            }
            if (stageEntry->detached.load())
            {
                CARB_LOG_INFO("[UNLOAD #%u] (caller=%s) Stage %" PRId64 " already detached - skipping",
                             unloadSeq, toString(caller), stageId);
                logCallerSymbol(unloadSeq);
                logCallStackForUnload(unloadSeq);
                return OVPHYSX_API_SUCCESS;
            }
            if (stageEntry->unloading.exchange(true))
            {
                CARB_LOG_INFO("[UNLOAD #%u] (caller=%s) Stage %" PRId64 " already unloading - skipping",
                             unloadSeq, toString(caller), stageId);
                return OVPHYSX_API_SUCCESS;
            }
            StageUnloadGuard unloadGuard(stageEntry);
            CARB_LOG_INFO("[UNLOAD #%u] (caller=%s) Detaching USD stage (stageId=%" PRId64 ")",
                         unloadSeq, toString(caller), stageId);

            cleanup_output_writes(*instance);
            ovphysx_sdf_view_cleanup_instance(instance);

            // Even when physics was never explicitly attached, an attach+detach pair
            // is needed. Scene population during load creates state that only
            // detachStage clears, and skipping it leaves dangling refs that corrupt
            // later simulate().
            {
                omni::physx::IPhysxSimulation* physxSim =
                    instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;

                if (physxSim)
                {
                    std::lock_guard<std::mutex> attachLock(g_gpuAttachMutex);
                    if (!instance->physics_attached.load(std::memory_order_acquire))
                    {
                        // Pair a quick attach+detach so the PhysX plugin releases
                        // any refs it accumulated during load. This attaches only
                        // this stage id; a peer instance's attachment stays up.
                        applyAttachTimeGpuSelection(
                            *instance, carb::getFramework(), "late attachStage for detach");
                        const omni::physics::tensors::AttachHandle lateAttach = physxSim->attachStage(stageId, {});
                        {
                            std::unique_lock<std::shared_mutex> liveLock(g_instances_mutex);
                            instance->attachHandle = lateAttach;
                        }
                        CARB_LOG_INFO(
                            "[UNLOAD #%u] Stage %" PRId64
                            " - late attach for clean detach (physics was deferred)",
                            unloadSeq, stageId);
                    }

                    CARB_LOG_INFO(
                        "[UNLOAD #%u] Detaching PhysX simulation for stage %" PRId64,
                        unloadSeq, stageId);
                    if (checkRuntimeResult(physxSim->detachStage(instance->attachHandle), "detachStage") != OVPHYSX_API_SUCCESS)
                        return OVPHYSX_API_ERROR;
                    ovphysx_object_change_detached_instance(handle);
                    finalize_recording_after_detach(physxSim, handle);
                    CARB_LOG_INFO(
                        "[UNLOAD #%u] PhysX simulation detached for stage %" PRId64,
                        unloadSeq, stageId);
                }
                else
                {
                    CARB_LOG_WARN(
                        "[UNLOAD #%u] WARNING: PhysX simulation interface unavailable for stage %" PRId64,
                        unloadSeq, stageId);
                }
            }
            
            
            // Release the tensor SimulationBackend's data for this attach before
            // clearing it, so the tensor backend does not hold stale views across
            // reset or reattach.
            if (omni::physics::tensors::TensorApi* tensorApi =
                    omni::physx::runtime::tryGetTensorApiInterface()) {
                if (tensorApi->resetStage)
                    tensorApi->resetStage(instance->attachHandle);
            }

            {
                std::unique_lock<std::shared_mutex> liveLock(g_instances_mutex);
                instance->attachedStageId = 0;
                instance->attachHandle = omni::physics::tensors::kNoAttach;
            }
            instance->resetStageFlags();
            unloadGuard.disarm();
            markStageDetached(stageId, "unload_usd_complete");
        } catch (const std::exception& e) {
                CARB_LOG_ERROR("[UNLOAD] Exception during stage detach: %s", e.what());
            return OVPHYSX_API_ERROR;
        } catch (...) {
            CARB_LOG_ERROR("[UNLOAD] Unknown exception during stage detach");
            return OVPHYSX_API_ERROR;
        }
    }
    
    return OVPHYSX_API_SUCCESS;
}

static void clearVisualizationScopeTokens(uint64_t attach)
{
    if (OvphysxSidecarSetVizScopeTokensFn clearScope =
            g_sidecarSetVizScopeTokens.load(std::memory_order_acquire))
    {
        (void)clearScope(attach, nullptr, 0u);
    }
}

static ovphysx_result_t detach_ovstage_body(ovphysx_instance_handle_t handle, InstanceData& instance);
static bool lifecycle_runs_on_caller();

static constexpr char kDestroyCancelled[] = "Task cancelled: the instance was destroyed before the task started";

ovphysx_api_status_t omni_sdk_physx_destroy(
    ovphysx_instance_handle_t handle, std::shared_ptr<ovphysx::CarboniteLoader>& deferredCarbonite)
{
    // Reject an already-absent handle before any process-global teardown. This
    // defensive path lets Python recover from an ambiguous FFI exception
    // without disturbing live peers or their asynchronous state.
    const std::shared_ptr<InstanceData> destroyed = get_instance(handle);
    if (!destroyed)
        return OVPHYSX_API_ERROR;
    ovphysx::InstanceExecutor* const executor = destroyed->executor.get();

    // The venue. The caller holds companion transition admission, which recording start
    // and stop need as well, so the recording state read here holds until destroy
    // returns. While a recording is active, destruction is a process-wide lifecycle call,
    // as attach and detach are then: it takes every instance's lane, its own preempted,
    // so no job of any instance runs while its teardown may finalize the recording.
    // Otherwise it preempts its own lane. Either way it waits only for a running job;
    // queued jobs stay queued. No other lock is held while it waits.
    bool everyLaneVenue = false;
    {
        std::lock_guard<std::mutex> recordingLock(g_recordingMutex);
        everyLaneVenue = lifecycle_runs_on_caller();
    }
    ovphysx::AllInstanceLanes everyLane;
    ovphysx::InstanceLane ownLane;
    const ovphysx_api_status_t venue = everyLaneVenue ? everyLane.acquire(handle) : ownLane.preempt(handle);
    if (venue != OVPHYSX_API_SUCCESS)
        return venue == OVPHYSX_API_INVALID_STATE ? venue : OVPHYSX_API_ERROR;

    // Retryable checks, with the rest of the queue intact. The recording owner finalizes
    // the recording, which needs the shared runtime idle: while it is busy, the owner's
    // destruction is refused, and returning releases the lanes, so every worker resumes
    // and nothing changed. The last live attachment of a recording that another instance
    // owns is torn down regardless, which finalizes the stream.
    bool recordingOwnedAtDestroy = false;
    if (everyLaneVenue)
    {
        {
            std::lock_guard<std::mutex> recordingLock(g_recordingMutex);
            recordingOwnedAtDestroy = g_omniPvdRecordingOwner == handle;
        }
        if (recordingOwnedAtDestroy &&
            (companionRuntime().hasPendingPreviews() || omni::physx::runtime::isRuntimeExecuting()))
            return set_error(OVPHYSX_API_INVALID_STATE,
                             "Destroying the recording owner needs the shared runtime idle: fetch companion "
                             "previews and retry")
                .status;
    }

    // Teardown is committed. Queued jobs that never started are cancelled: their receipts
    // fail, and report NOT_FOUND once the instance is gone. Then the worker is joined.
    if (executor)
    {
        executor->cancelQueued(kDestroyCancelled);
        executor->stopAndJoin();
    }
    // No job can use an output session any more.
    cleanup_output_sessions(handle, *destroyed);

    // Tearing the attachment down is lifecycle work: it never overlaps another
    // instance's attach or detach job. The lock comes after every lane and the join.
    std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);

    bool detachOvstage = false;
    // Capture the attach handle before detach/unload clear it. It is the TensorAPI
    // cleanup fallback when unload exits before clearing the stage.
    omni::physics::tensors::AttachHandle destroyedAttachHandle = omni::physics::tensors::kNoAttach;
    {
        std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
        auto it = g_instances.find(handle);
        if (it != g_instances.end() && it->second)
        {
            destroyedAttachHandle = it->second->attachHandle;
            detachOvstage = it->second->ovstage_attached;
        }
    }
    if (detachOvstage)
    {
        // The detach body runs under the lane(s), the companion transition and the
        // lifecycle lock that destroy holds.
        const std::shared_ptr<InstanceData> attached = get_instance(handle);
        const ovphysx_result_t detachResult =
            attached ? detach_ovstage_body(handle, *attached)
                     : set_error(OVPHYSX_API_INVALID_ARGUMENT, "detach_ovstage: invalid handle");
        if (detachResult.status != OVPHYSX_API_SUCCESS)
        {
            // Failed pending work can prevent the normal detach path from
            // running. The scope owns dictionary-backed tokens, so clear it
            // before the instance (and potentially the dictionary) disappears.
            // A detach that already dropped the scope and then failed detachStage
            // must not clear again.
            std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
            if (!instanceShared || !instanceShared->visualizationScopeDropped)
            {
                if (instanceShared)
                    instanceShared->visualizationScopeDropped = true;
                clearVisualizationScopeTokens(destroyedAttachHandle);
            }
            omni::physx::ovxDebugRenderNoteAttach(handle, 0, 1);

            // ovphysx_detach_ovstage() bailed out before its runtime-detach block
            // (for example a companion transition refused it), so this handle may
            // still hold a live runtime attachment. Force that attachment down,
            // independently of pending-op success, so destruction never leaves it
            // owned by a handle that no longer exists.
            bool clearLiveAttachment = false;
            {
                std::lock_guard<std::mutex> attachLock(g_gpuAttachMutex);
                if (instanceShared && instanceShared->attachHandle != omni::physics::tensors::kNoAttach)
                {
                    omni::physx::IPhysxSimulation* physxSim =
                        instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
                    if (physxSim && physxSim->detachStage)
                    {
                        // Same order as ovphysx_detach_ovstage. resetStage has to
                        // see the live handle; detachStage destroys the stage the
                        // tensor backend borrows. Clearing the handle first makes
                        // unload's reset and the later destroyedAttachHandle reset
                        // both skip this attachment.
                        if (omni::physics::tensors::TensorApi* tensorApi =
                                omni::physx::runtime::tryGetTensorApiInterface())
                        {
                            if (tensorApi->resetStage)
                                tensorApi->resetStage(instanceShared->attachHandle);
                        }
                        physxSim->detachStage(instanceShared->attachHandle);
                        ovphysx_object_change_detached_instance(handle);
                        finalize_recording_after_detach(physxSim, handle);
                    }
                    clearLiveAttachment = true;
                }
            }
            if (clearLiveAttachment)
            {
                // Publish the clear under the same mutex the live-attachment
                // scans use. Do this after releasing g_gpuAttachMutex: recording
                // finalization already takes the instances mutex while holding it.
                std::unique_lock<std::shared_mutex> liveLock(g_instances_mutex);
                instanceShared->attachHandle = omni::physics::tensors::kNoAttach;
                instanceShared->ovstage_attached = false;
            }
        }
    }

    {
        ScopedUnloadCaller callerScope(UnloadCaller::kDestroy);
        omni_sdk_physx_unload_usd(handle);  // Ignore return value - continue cleanup even if unload fails
    }

    // Stage teardown releases PxPhysics while sampling is still active so its
    // final object-removal telemetry reaches the stream. If no stage existed,
    // or teardown failed before doing that, finalize the owner session here.
    std::shared_ptr<InstanceData> recordingInstance = get_instance(handle);
    {
        std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
        if (recordingOwnedAtDestroy && recordingInstance && recordingInstance->carbonite &&
            (g_omniPvdRecordingOwner == handle ||
             g_omniPvdRecordingOwner == OVPHYSX_INVALID_HANDLE))
        {
            omni::physx::IPhysxSimulation* physxSim = recordingInstance->carbonite->getPhysxSimulation();
            if (physxSim && physxSim->stopOmniPvdRecording && physxSim->getOmniPvdRecordingStatus &&
                recordingActive(physxSim))
            {
                // If detach cleared the latch but did not actually stop sampling,
                // restore ownership until the explicit finalizer succeeds.
                setRecordingOwner(handle);
                const omni::physx::OmniPvdRecordingResult stopResult = physxSim->stopOmniPvdRecording();
                if (stopResult != omni::physx::OmniPvdRecordingResult::eSuccess)
                    CARB_LOG_ERROR("[DESTROY] Failed to finalize the active OmniPVD recording");
            }
            reconcile_recording_owner_after_runtime_transition(physxSim);
        }
    }

    bool stageUnloadCompleted = false;
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        auto it = g_instances.find(handle);
        stageUnloadCompleted = it != g_instances.end() && it->second &&
                               it->second->attachHandle == omni::physics::tensors::kNoAttach;
    }


    ovphysx_object_change_cleanup_instance(handle);

    // Remove from instance registry first, then clean up outside the map lock.
    std::shared_ptr<InstanceData> instanceShared;
    bool lastInstance = false;
    {
        std::unique_lock<std::shared_mutex> map_lock(g_instances_mutex);
        auto it = g_instances.find(handle);
        if (it == g_instances.end())
        {
            return OVPHYSX_API_ERROR;  // Already destroyed?
        }
        instanceShared = std::move(it->second);
        g_instances.erase(it);
        lastInstance = g_instances.empty();
    }

    // Explicitly reset components in order to control destruction.
    if (instanceShared)
    {
        try
        {
            // Clean up contact, tensor, and SDF bindings before destroying TensorAPI views / Carbonite.
            ovphysx_contact_binding_cleanup_instance(instanceShared.get());
            ovphysx_tensor_binding_cleanup_instance(instanceShared.get());
            ovphysx_sdf_view_cleanup_instance(instanceShared.get());

            // SimulationBackend per-stage reset happens in the tensor reset block below.
        }
        catch (const std::exception& e)
        {
            CARB_LOG_ERROR("[DESTROY] Exception during component cleanup: %s", e.what());
        }
        catch (...)
        {
            CARB_LOG_ERROR("[DESTROY] Unknown exception during component cleanup");
        }

        // Keep Carbonite alive until the public API boundary zone has closed. The
        // last loader shuts down and deregisters the profiler, so releasing it here
        // would prevent that zone's RAII destructor from sending its matching end.
        deferredCarbonite = std::move(instanceShared->carbonite);
    }

    omni::physics::tensors::TensorApi* tensorApi = omni::physx::runtime::tryGetTensorApiInterface();
    if (tensorApi)
    {
        // Fallback for this attachment when unload did not clear its tensor data.
        if (tensorApi->resetStage && destroyedAttachHandle != omni::physics::tensors::kNoAttach &&
            !stageUnloadCompleted)
            tensorApi->resetStage(destroyedAttachHandle);
    }

    if (lastInstance)
    {
        CARB_LOG_INFO("[ovphysx] Direct runtime retained until ovphysx_shutdown");
    }

    return OVPHYSX_API_SUCCESS;
}
// ==================== Public API functions ====================

// Main SDK bootstrap entry point.
// It serializes process-global setup, starts Carbonite, applies user config,
// then loads ovphysx's own PhysX plugins. It rejects a process that already
// loaded a PhysX Carbonite stack. ovphysx neither loads nor registers USD here:
// the application owns USD and schema registration.
static ovphysx_result_t createInstanceInternal(const ovphysx_create_args* create_args, ovphysx_instance_handle_t* out_handle) {
    if (!create_args || !out_handle) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments to create_instance");
    }
    if (create_args->config_entry_count > 0 && !create_args->config_entries) {
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT,
            "Invalid arguments to create_instance: config_entry_count is nonzero but config_entries is null");
    }

    ovphysx_create_args args_copy = *create_args;
    const ovphysx_create_args* args = &args_copy;

    // Create-time entries that reach live attachments act on every instance. Their
    // lanes come before the create lock: a callback running under a held lane may
    // create an instance, so nothing waits for a lane while holding that lock. Only
    // creation registers instances, so the registered set cannot grow while it is held.
    bool needsEveryLane = false;
    for (uint32_t i = 0; i < args->config_entry_count && !needsEveryLane; ++i)
        needsEveryLane = entryNeedsEveryLane(args->config_entries[i]);
    ovphysx::AllInstanceLanes everyLane;
    std::unique_lock<std::mutex> createLock(g_createInstanceMutex, std::defer_lock);
    for (;;)
    {
        if (needsEveryLane)
        {
            const ovphysx_api_status_t laneStatus = everyLane.acquire();
            if (laneStatus == OVPHYSX_API_INVALID_STATE)
                return {laneStatus};
            if (laneStatus != OVPHYSX_API_SUCCESS)
                return set_error(laneStatus, "create_instance: failed to finish earlier work on live instances");
        }
        createLock.lock();
        if (!needsEveryLane || everyLane.coversRegistered())
            break;
        createLock.unlock();
    }

    for (uint32_t i = 0; i < args->config_entry_count; ++i)
    {
        if (isRetiredOvdConfigEntry(args->config_entries[i]))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, kRetiredOvdConfigError);
    }

    auto instanceData = std::make_shared<InstanceData>();


    // Parse active_cuda_gpus early to fail fast before bringing up Carbonite/PhysX.
    // The ordinals are range-validated post-init and stored on the instance for the
    // deferred /physics/cudaDevice write at scene attach. CarboniteLoader gets the
    // fixed -2 sentinel below, not an ordinal from this list.
    std::vector<int32_t> requestedOrdinals;
    {
        char parseErr[256] = {};
        const char* gpuStr = (args && args->active_cuda_gpus.ptr) ? args->active_cuda_gpus.ptr : nullptr;
        const size_t gpuLen = (args && args->active_cuda_gpus.ptr) ? args->active_cuda_gpus.length : 0;
        requestedOrdinals = parseActiveCudaGpus(gpuStr, gpuLen, parseErr, sizeof(parseErr));
        if (parseErr[0] != '\0')
        {
            CARB_LOG_ERROR("[ovphysx] active_cuda_gpus parse error: %s", parseErr);
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, parseErr);
        }
        // parseActiveCudaGpus only returns an empty vector on error, which is
        // handled above. Defensive guard.
        if (requestedOrdinals.empty())
        {
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "active_cuda_gpus: internal parse error (empty ordinal list)");
        }
    }

    // ========================================================================
    // Configure Carbonite bootstrap before PhysX plugins load. The PhysX CUDA
    // ordinal is applied later by the scene-attach paths.
    // ========================================================================
    {
        if (args && args->config_entry_count > 0 && args->config_entries)
        {
            for (uint32_t i = 0; i < args->config_entry_count; ++i)
            {
                const auto& entry = args->config_entries[i];
                const char* path = getConfigEntryPath(entry);
                std::string carbonitePath;
                if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_CARBONITE && entry.key.carbonite_key.ptr)
                {
                    carbonitePath.assign(entry.key.carbonite_key.ptr, entry.key.carbonite_key.length);
                    path = carbonitePath.c_str();
                }
                if (!path) continue;
                if (strcmp(path, "/physics/cudaDevice") == 0)
                {
                    CARB_LOG_ERROR("[Config] Cannot set '/physics/cudaDevice' via config entries. Use active_cuda_gpus on create_args instead.");
                    return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Cannot set /physics/cudaDevice via config entries. Use active_cuda_gpus instead.");
                }
            }
        }

        // Note: /physics/suppressReadback (DirectGPU-API mode) is opt-in by the
        // host and ovphysx never writes it. See the create_args doc comment in
        // ovphysx_types.h.
    }

    // Log level is configured globally via ovphysx_set_log_level() (managed by LogManager).
    // CarboniteLoader::initialize() calls onCarboniteLoggingReady() to apply it.

    // CarboniteLoader auto-detects bundled deps via getLibraryDirectory()
    instanceData->carbonite = std::make_shared<ovphysx::CarboniteLoader>();
    if (!instanceData->carbonite->initialize()) {
        const std::string& loaderError = instanceData->carbonite->getLastError();
        const char* message = loaderError.empty() ? "Failed to initialize Carbonite and load PhysX plugins"
                                                  : loaderError.c_str();
        CARB_LOG_ERROR("%s", message);
        return set_error(OVPHYSX_API_ERROR, message);
    }

    // Apply process-global capability before PhysX plugins create the runtime.
    {
        carb::Framework* framework = carb::getFramework();
        carb::settings::ISettings* settings =
            framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
        if (settings)
        {
            // Canonical process-global configuration is independent of instance
            // creation. Apply values stored before Carbonite became available.
            applyPendingGlobalConfig(settings);
            const char* queuedError = nullptr;
            const ovphysx_api_status_t queuedStatus = applyQueuedConfigEntries(settings, queuedError);
            if (queuedStatus != OVPHYSX_API_SUCCESS)
                return set_error(queuedStatus, queuedError ? queuedError : "Invalid config entry");
            if (args->config_entry_count > 0)
            {
                for (uint32_t i = 0; i < args->config_entry_count; ++i)
                {
                    const ovphysx_api_status_t workerStatus =
                        validateWorkerPoolStartupEntry(args->config_entries[i]);
                    if (workerStatus != OVPHYSX_API_SUCCESS)
                        return set_error(workerStatus, "Invalid worker pool startup config");
                    if (isWorkerPoolStartupEntry(args->config_entries[i]) && isTaskingStarted())
                    {
                        return set_error(
                            OVPHYSX_API_ERROR,
                            "Worker pool startup config cannot change after carb.tasking has started");
                    }
                }
                CARB_LOG_INFO(
                    "[ovphysx] Applying %u user config entries (pre-plugin-load)",
                    args->config_entry_count);
                const char* entryError = nullptr;
                const ovphysx_api_status_t applyStatus =
                    applyCreateConfigEntries(settings, args->config_entries, args->config_entry_count, entryError);
                if (applyStatus != OVPHYSX_API_SUCCESS)
                    return set_error(applyStatus, entryError ? entryError : "Invalid config entry");
                everyLane.release();
            }

            // Report the DirectGPU mode now that any create-config overrides have been applied.
            // CarboniteLoader logs /physics/suppressReadback once per process at bootstrap, before this
            // point, so a developer opting into DirectGPU through create-config would otherwise read
            // only that pre-override "unset" snapshot. The setting is process-wide and a GPU scene
            // reads it when it is created at attach, so this reports the value after this instance's
            // create-config; a later instance's create-config or the host can still change it before
            // this instance's scenes attach.
            const bool suppressReadback = settings->getAsBool("/physics/suppressReadback");
            CARB_LOG_INFO("[ovphysx] /physics/suppressReadback=%s after create-config (process-wide); "
                          "DirectGPU %s for GPU scenes attached while it holds.",
                          suppressReadback ? "true" : "false", suppressReadback ? "enabled" : "disabled");
        }
        else
        {
            CARB_LOG_WARN("[ovphysx] Warning: ISettings not available, cannot apply user config entries");
        }
    }

    // Resolve the profiler backend before the PhysX plugins load: CarboniteLoader binds
    // the backend while loading them, and a selection made after that would arrive too
    // late for the zones it was meant to record. This call also writes the
    // OVPHYSX_PROFILER and OVPHYSX_PROFILER_OUTPUT environment variables through to
    // their settings.
    ovphysx::profiling::resolveBackend();

    if (!instanceData->carbonite->loadPhysxPlugins()) {
        const std::string& loaderError = instanceData->carbonite->getLastError();
        const char* message = loaderError.empty() ? "Failed to load PhysX plugins"
                                                  : loaderError.c_str();
        CARB_LOG_ERROR("%s", message);
        return set_error(OVPHYSX_API_ERROR, message);
    }
    // The PhysX plugins started carb.tasking; executor waits can now suspend tasks.
    installExecutorWaitHooks();
    // Preload the internal sidecar so its carb::Framework + OmniCore built-ins are seeded
    // before first use.
    CARB_LOG_INFO("[ovphysx] Attempting to preload internal sidecar...");
    if (loadInternalSidecar()) {
        CARB_LOG_INFO("[ovphysx] Internal sidecar preloaded successfully");
    } else {
        CARB_LOG_INFO("[ovphysx] Internal sidecar preload failed (will load on-demand on first use)");
    }

    // Validate the active_cuda_gpus ordinal early so the error surfaces at create
    // time rather than at first attach. The /physics/cudaDevice write is deferred to
    // the scene-attach path under g_gpuAttachMutex so it is atomic with PhysX attachment.
    if (args && args->active_cuda_gpus.ptr && args->active_cuda_gpus.length > 0 &&
        !isProcessGpuDisabled() && omni::physx::cudaShim::isCudaAvailable())
    {
        // Validate multi-GPU patterns at create time so callers get a fast,
        // clear error rather than a silent single-GPU fallback at attach time.
        char validationErr[384] = {};
        if (!validateActiveCudaGpuOrdinals(requestedOrdinals, validationErr, sizeof(validationErr)))
        {
            CARB_LOG_ERROR("[ovphysx] %s", validationErr);
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, validationErr);
        }
    }

    // Store creation arguments for introspection
    if (args) {
        instanceData->create_args = *args;
        // Clear pointers to caller-owned memory so nothing in create_args dangles
        // after this call returns. config_entries were applied above and
        // active_cuda_gpus lives on as the parsed active_cuda_ordinals.
        instanceData->create_args.config_entries = nullptr;
        instanceData->create_args.config_entry_count = 0;
        instanceData->create_args.active_cuda_gpus.ptr = nullptr;
        instanceData->create_args.active_cuda_gpus.length = 0;
        if (args->active_cuda_gpus.ptr)
        {
            instanceData->active_cuda_gpus.assign(
                args->active_cuda_gpus.ptr,
                args->active_cuda_gpus.length);
            instanceData->active_cuda_gpus_configured = true;
        }
        // Persist the parsed ordinals only when the caller restricted GPU ordinals.
        // requestedOrdinals defaults to {0} for empty input, so gate on the original
        // string to keep "no active_cuda_gpus" distinct from an explicit "0".
        if (args->active_cuda_gpus.ptr && args->active_cuda_gpus.length > 0) {
            instanceData->active_cuda_ordinals = requestedOrdinals;
        }
    } else {
        instanceData->create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    }

    const ovphysx_instance_handle_t handle = ovphysx::internal::allocateOpaqueObjectHandle();
    if (handle == OVPHYSX_INVALID_HANDLE)
        return set_error(OVPHYSX_API_ERROR, "opaque object handle space exhausted");

    // Summarize the process GPU policy at create so hosts can confirm the effective
    // CPU-only mode and CUDA selection intent. Per-scene enableGPUDynamics remains
    // USD-owned (ADR-0011). This reports hard-policy and create-args intent only,
    // not the attach-time resolved dynamics mode or the chosen ordinal.
    {
        const bool processCpuOnly = isProcessGpuDisabled();
        const bool envDisabled = isEnvGpuDisabled();
        const bool apiForced = g_forceCpuMode.load(std::memory_order_acquire);
        const char* cpuReason = "none";
        if (envDisabled && apiForced)
            cpuReason = "OVPHYSX_DISABLE_GPU+ovphysx_set_cpu_mode";
        else if (envDisabled)
            cpuReason = "OVPHYSX_DISABLE_GPU";
        else if (apiForced)
            cpuReason = "ovphysx_set_cpu_mode";

        const char* cudaAvailableStr = "n/a";
        if (!processCpuOnly)
            cudaAvailableStr = omni::physx::cudaShim::isCudaAvailable() ? "true" : "false";

        char activeGpusBuf[128] = "inactive";
        if (!processCpuOnly)
        {
            const std::vector<int32_t>& ordinals = instanceData->active_cuda_ordinals;
            if (ordinals.empty())
            {
                // Empty active_cuda_gpus is "no ovphysx ordinal override"
                // (ADR-0011), not PhysX automatic selection ("-1").
                std::snprintf(activeGpusBuf, sizeof(activeGpusBuf), "no_override");
            }
            else
            {
                size_t off = 0;
                activeGpusBuf[0] = '\0';
                for (size_t i = 0; i < ordinals.size(); ++i)
                {
                    const int written = std::snprintf(
                        activeGpusBuf + off, sizeof(activeGpusBuf) - off, "%s%d",
                        (i == 0) ? "" : ",", ordinals[i]);
                    if (written < 0 || static_cast<size_t>(written) >= sizeof(activeGpusBuf) - off)
                        break;
                    off += static_cast<size_t>(written);
                }
            }
        }

        CARB_LOG_INFO(
            "[ovphysx] Instance %" PRIu64
            " created: process_cpu_only=%s [%s] cuda_available=%s active_cuda_gpus=%s",
            handle, processCpuOnly ? "true" : "false", cpuReason, cudaAvailableStr, activeGpusBuf);
    }

    // Where this instance runs short async calls and steps, for tests only (see
    // InstanceData::test_execution).
    instanceData->test_execution = g_testExecution.load(std::memory_order_relaxed);

    // Start the instance worker last, so no earlier failure leaves a parked thread.
    try {
        instanceData->executor = createInstanceExecutor();
    } catch (const std::exception& error) {
        return set_error(OVPHYSX_API_ERROR,
                         std::string("Failed to start the instance worker thread (set "
                                     "OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION to false to create instances "
                                     "without one): ") +
                             error.what());
    }

    try {
        std::unique_lock<std::shared_mutex> map_lock(g_instances_mutex);
        g_instances[handle] = instanceData;
    } catch (const std::exception& error) {
        // Nothing else would ever stop the worker of an instance that was never registered.
        instanceData->executor->stopAndJoin();
        return set_error(OVPHYSX_API_ERROR, std::string("Failed to register the instance: ") + error.what());
    }
    *out_handle = handle;
    g_runtimeConfigStarted.store(true, std::memory_order_release);

    return success();
}

// Process-wide lifecycle: ovphysx_initialize() / ovphysx_shutdown() / the
// init-check in ovphysx_create_instance().
//
// Per the ovphysx threading contract, process-lifecycle calls are serialized by
// the caller (the Python wrapper uses its lifecycle condition). The g_sdkLifecycle.initialized
// atomic only enforces matched init/shutdown pairing, it is not a lock. These are
// not guarded with g_createInstanceMutex because createInstanceInternal() already
// takes that non-recursive mutex, so checking g_sdkLifecycle.initialized under it would
// self-deadlock.
//
// Initialize performs only CPU capability validation, the atomic lifecycle claim,
// and environment-policy latching. It must not emit logs, invoke callbacks, or
// wait for callback delivery, because the Python wrapper lets concurrent
// constructors wait for this transition. Shutdown can flush and drain callbacks,
// so Python callers racing final shutdown remain fail-fast.
OVPHYSX_API ovphysx_result_t ovphysx_initialize(void)
{
    OVPHYSX_API_PROFILE_ZONE(initialize);
#if defined(__x86_64__) || defined(_M_X64) || defined(__amd64__)
    if (!ovphysx::internal::cpuSupportsAvx())
    {
        return set_error(
            OVPHYSX_API_ERROR,
            "This CPU does not support AVX, which is required by ovphysx x86_64 builds. "
            "Use an x86_64 CPU with AVX enabled, or on ARM Linux use the aarch64 wheel.");
    }
#endif

    bool expected = false;
    if (!g_sdkLifecycle.initialized.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    {
        return set_error(OVPHYSX_API_ERROR, "ovphysx_initialize called while already initialized");
    }

    // Sample OVPHYSX_DISABLE_GPU at the documented initialization boundary so a
    // prior observational get_cpu_mode() cannot permanently miss a later setenv.
    latchEnvGpuDisabledFromEnvironment();

    {
        std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
        companionRuntime().setAdmissionOpen(true);
    }

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_create_instance(ovphysx_instance_handle_t* out_handle)
{
    OVPHYSX_API_PROFILE_ZONE(create_instance);
    if (!g_sdkLifecycle.initialized.load(std::memory_order_acquire))
    {
        if (out_handle)
        {
            *out_handle = OVPHYSX_INVALID_HANDLE;
        }
        return set_error(OVPHYSX_API_ERROR, "ovphysx_create_instance called before ovphysx_initialize");
    }

    const ovphysx_create_args createArgs = OVPHYSX_CREATE_ARGS_DEFAULT;
    return createInstanceInternal(&createArgs, out_handle);
}

OVPHYSX_API ovphysx_result_t ovphysx_create_instance_with_args(
    const ovphysx_create_args* create_args,
    ovphysx_instance_handle_t* out_handle)
{
    OVPHYSX_API_PROFILE_ZONE(create_instance_with_args);
    if (!g_sdkLifecycle.initialized.load(std::memory_order_acquire))
    {
        if (out_handle)
        {
            *out_handle = OVPHYSX_INVALID_HANDLE;
        }
        return set_error(
            OVPHYSX_API_ERROR,
            "ovphysx_create_instance_with_args called before ovphysx_initialize");
    }

    return createInstanceInternal(create_args, out_handle);
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_instance(ovphysx_instance_handle_t handle) {
    OVPHYSX_API_PROFILE_ZONE(destroy_instance);
    // Destruction waits for the instance's work, so a callback, which may run under a
    // held lane, and the instance's own worker are refused before anything changes.
    if (ovphysx::refuseBlockingReentry(get_instance(handle).get()) != OVPHYSX_API_SUCCESS)
        return {OVPHYSX_API_INVALID_STATE};
    CompanionTransitionGuard companionTransition(companionRuntime());
    if (!companionTransition)
        return set_error(OVPHYSX_API_INVALID_STATE, "Companion runtime operation is executing; retry transition");
    if (companionRuntime().hasPendingPreviews())
    {
        std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
        const std::shared_ptr<InstanceData> owner = get_instance(handle);
        const bool lastLiveAttachment = owner &&
            owner->attachHandle != omni::physics::tensors::kNoAttach && !other_instance_attached(handle);
        if (g_omniPvdRecordingOwner != OVPHYSX_INVALID_HANDLE &&
            (g_omniPvdRecordingOwner == handle || lastLiveAttachment))
            return set_error(OVPHYSX_API_INVALID_STATE, "Fetch companion preview results before recording-owner destruction");
    }
    // Reserve destruction against new companion leases before any mutation.
    std::shared_ptr<InstanceData> companionOwner;
    {
        std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
        const std::unordered_map<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>::iterator it = g_instances.find(handle);
        if (it != g_instances.end())
        {
            companionOwner = it->second;
            if (companionOwner->companionLeases || companionOwner->companionDestroying)
                return set_error(OVPHYSX_API_INVALID_STATE, "Owner is busy: close companion sessions before destruction");
            companionOwner->companionDestroying = true;
        }
    }
    // No g_createInstanceMutex here. omni_sdk_physx_destroy has its own
    // serialization, and ovphysx_destroy_instance is called from PhysX RAII
    // destructors that can fire on error paths inside createInstanceInternal,
    // which holds that mutex, so taking it here would deadlock.
    std::shared_ptr<ovphysx::CarboniteLoader> deferredCarbonite;
    ovphysx_api_status_t status = OVPHYSX_API_ERROR;
    try
    {
        status = omni_sdk_physx_destroy(handle, deferredCarbonite);
    }
    catch (...)
    {
        if (companionOwner)
        {
            std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
            companionOwner->companionDestroying = false;
        }
        CARB_LOG_ERROR("[DESTROY] Unexpected exception escaped internal teardown");
        OVPHYSX_API_PROFILE_ZONE_END();
        deferredCarbonite.reset();
        return set_error(OVPHYSX_API_ERROR, "ovphysx_destroy_instance: exception during teardown");
    }

    // The final CarboniteLoader owns profiler shutdown. End this API zone while
    // its backend is still registered, then release the loader and flush the trace.
    OVPHYSX_API_PROFILE_ZONE_END();
    deferredCarbonite.reset();

    if (status != OVPHYSX_API_SUCCESS) {
        if (companionOwner)
        {
            std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
            companionOwner->companionDestroying = false;
        }
        return {status}; // Don't set error on destroy failure
    }
    return success();
}

namespace
{
bool copy_capture_paths(const ovphysx_string_t* paths, uint32_t count, std::vector<std::string>& copied)
{
    if (count && !paths)
        return false;
    copied.reserve(count);
    for (uint32_t index = 0; index < count; ++index)
    {
        std::string path;
        if (!omni::physx::copyOmniPvdConfigString(paths[index].ptr, paths[index].length, path) ||
            !omni::physx::looksLikeAbsolutePrimPath(path, false))
            return false;
        copied.push_back(std::move(path));
    }
    return true;
}
}

OVPHYSX_API ovphysx_result_t ovphysx_ovd_start_recording(
    ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination,
    const ovphysx_omnipvd_recording_options_t* capture)
{
    OVPHYSX_API_PROFILE_ZONE(ovd_start_recording);
    const bool discovery = capture && capture->wait_for_selection;
    const char* operation = "start_recording: ";
    if (!destination)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "start_recording: destination is null");
    if (discovery && destination->transport != OVPHYSX_OMNIPVD_TRANSPORT_TCP)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "start_recording: wait_for_selection requires a TCP destination");
    if (ovphysx::refuseBlockingReentry(nullptr) != OVPHYSX_API_SUCCESS)
        return {OVPHYSX_API_INVALID_STATE};
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, std::string(operation) + "invalid handle");
    std::vector<std::string> scenePaths;
    std::vector<std::string> objectPaths;
    if (capture &&
        (!copy_capture_paths(capture->scene_paths, capture->scene_count, scenePaths) ||
         !copy_capture_paths(capture->object_paths, capture->object_count, objectPaths)))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, std::string(operation) + "invalid filter path lists");

    std::string filePath;
    std::string tcpAddress;
    if (!omni::physx::copyOmniPvdConfigString(
            destination->file_path.ptr, destination->file_path.length, filePath) ||
        !omni::physx::copyOmniPvdConfigString(
            destination->tcp_address.ptr, destination->tcp_address.length, tcpAddress))
    {
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT, std::string(operation) + "destination strings must not contain embedded NUL bytes");
    }

    uint32_t transport = 0;
    const char* target = nullptr;
    uint16_t tcpPort = 0;
    uint32_t tcpTimeoutMs = 0;
    if (destination->transport == OVPHYSX_OMNIPVD_TRANSPORT_FILE)
    {
        if (filePath.empty() || !tcpAddress.empty() || destination->tcp_port != 0 ||
            destination->tcp_timeout_ms != 0)
        {
            return set_error(
                OVPHYSX_API_INVALID_ARGUMENT,
                std::string(operation) + "FILE requires a non-empty file_path and empty/zero TCP fields");
        }
        target = filePath.c_str();
    }
    else if (destination->transport == OVPHYSX_OMNIPVD_TRANSPORT_TCP)
    {
        if (!filePath.empty() || tcpAddress.empty() || destination->tcp_port == 0 ||
            destination->tcp_port > 65535 || destination->tcp_timeout_ms < 0)
        {
            return set_error(
                OVPHYSX_API_INVALID_ARGUMENT,
                std::string(operation) + "TCP requires an empty file_path, non-empty address, port in 1..65535, and non-negative timeout");
        }
        transport = 1;
        target = tcpAddress.c_str();
        tcpPort = static_cast<uint16_t>(destination->tcp_port);
        tcpTimeoutMs = static_cast<uint32_t>(destination->tcp_timeout_ms);
    }
    else
    {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, std::string(operation) + "invalid destination transport");
    }

    // From here until this start finished, attach and detach run on their calling
    // threads, as they do while the recording is active. One queued before this point is
    // a job that the lanes below wait for, so no attach or detach job runs while a
    // recording is active, and finalization follows submission order. Declared before
    // the lanes, so it ends after they are released.
    struct PendingStart
    {
        PendingStart()
        {
            std::lock_guard<std::mutex> lock(g_recordingMutex);
            ++g_recordingStartsPending;
        }
        ~PendingStart()
        {
            std::lock_guard<std::mutex> lock(g_recordingMutex);
            --g_recordingStartsPending;
        }
    } pendingStart;
    ovphysx::AllInstanceLanes everyLane;
    const ovphysx_api_status_t waitStatus = everyLane.acquire();
    if (waitStatus != OVPHYSX_API_SUCCESS)
        return set_error(waitStatus, "start_recording: failed to reach a shared-runtime safe point");
    CompanionTransitionGuard companionTransition(companionRuntime());
    if (!companionTransition)
        return set_error(OVPHYSX_API_INVALID_STATE, "Companion runtime operation is executing; retry transition");
    std::unique_lock<std::mutex> recordingLock(g_gpuAttachMutex, std::defer_lock);
    if (enter_shared_runtime_safe_point(recordingLock) != OVPHYSX_API_SUCCESS)
        return set_error(OVPHYSX_API_INVALID_STATE, "start_recording: failed to reach a shared-runtime safe point");

    omni::physx::IPhysxSimulation* physxSim =
        instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || (discovery ? !physxSim->startOmniPvdDiscoveryWithFilterOwner :
                                    !physxSim->startOmniPvdRecordingWithFilterOwner))
        return set_error(OVPHYSX_API_ERROR, std::string(operation) + "OmniPVD runtime API is unavailable");

    bool recording = false;
    if (!physxSim->getOmniPvdRecordingStatus ||
        physxSim->getOmniPvdRecordingStatus(&recording) != omni::physx::RuntimeResult::eSuccess)
        return set_error(OVPHYSX_API_ERROR, std::string(operation) + "runtime status query failed");
    if (recording)
        return set_error(OVPHYSX_API_INVALID_STATE, std::string(operation) + "a recording is already active");

    if (discovery && instance->attachHandle == omni::physics::kNoAttach)
        return set_error(OVPHYSX_API_INVALID_STATE, "start_recording: wait_for_selection requires the caller's live attachment");

    // Resolve only after draining shared work and locking attachment transitions.
    // Public runtime lookups acquire their own admission, before recording starts.
    ::physx::PxOmniPvdFilterDesc nativeCapture;
    std::vector<const ::physx::PxScene*> scenes;
    std::vector<const ::physx::PxBase*> objects;
    if (capture)
    {
        nativeCapture.recordSecondaryData = capture->record_secondary_data;
        if (!scenePaths.empty() || !objectPaths.empty())
        {
            if (instance->attachHandle == omni::physics::kNoAttach)
                return set_error(OVPHYSX_API_INVALID_STATE, std::string(operation) + "capture paths require the caller's attachment");
            omni::physx::IPhysx* physxInterface = omni::physx::runtime::tryGetPhysxInterface();
            if (!physxInterface || !physxInterface->resolveObjectKey || !physxInterface->getPhysXPtr)
                return set_error(OVPHYSX_API_ERROR, std::string(operation) + "attachment lookup is unavailable");
            const omni::physics::AttachHandle attach = instance->attachHandle;
            for (const std::string& path : scenePaths)
            {
                const omni::physics::parse::ObjectKey key = physxInterface->resolveObjectKey(attach, path.c_str());
                const ::physx::PxScene* scene = static_cast<const ::physx::PxScene*>(
                    physxInterface->getPhysXPtr(attach, key, omni::physx::ePTScene));
                if (!scene)
                    return set_error(OVPHYSX_API_NOT_FOUND, std::string(operation) + "scene path has no live physics scene in this attachment");
                scenes.push_back(scene);
            }
            if (!objectPaths.empty())
            {
                const omni::physx::RuntimeResult resolved =
                    omni::physx::omniPvdResolveCaptureObjects(attach, objectPaths, objects);
                if (resolved == omni::physx::RuntimeResult::eBusy)
                    return set_error(OVPHYSX_API_INVALID_STATE, std::string(operation) + "object lookup is busy; retry transition");
                if (resolved != omni::physx::RuntimeResult::eSuccess)
                    return set_error(OVPHYSX_API_NOT_FOUND,
                                     std::string(operation) + "object path names no supported recording object in this attachment");
            }
        }
        nativeCapture.scenes = scenes.data();
        nativeCapture.nbScenes = static_cast<::physx::PxU32>(scenes.size());
        nativeCapture.objects = objects.data();
        nativeCapture.nbObjects = static_cast<::physx::PxU32>(objects.size());
    }

    const omni::physx::OmniPvdRecordingResult result = discovery ?
        physxSim->startOmniPvdDiscoveryWithFilterOwner(target, tcpPort, tcpTimeoutMs,
                                                     capture ? &nativeCapture : nullptr, instance->attachHandle) :
        physxSim->startOmniPvdRecordingWithFilterOwner(transport, target, tcpPort, tcpTimeoutMs,
                                                     capture ? &nativeCapture : nullptr, instance->attachHandle);
    switch (result)
    {
    case omni::physx::OmniPvdRecordingResult::eSuccess:
        setRecordingOwner(handle);
        return success();
    case omni::physx::OmniPvdRecordingResult::eInvalidState:
        return set_error(
            OVPHYSX_API_INVALID_STATE,
            std::string(operation) + "attach and initialize a stage before starting a late recording");
    case omni::physx::OmniPvdRecordingResult::eNotSupported:
        return set_error(OVPHYSX_API_NOT_IMPLEMENTED, std::string(operation) + "OmniPVD recording is unsupported on this platform");
    case omni::physx::OmniPvdRecordingResult::eNotCapable:
        return set_error(
            OVPHYSX_API_INVALID_STATE,
            std::string(operation) + "set OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE=true before creating the first instance");
    case omni::physx::OmniPvdRecordingResult::eBusy:
        return set_error(OVPHYSX_API_INVALID_STATE, std::string(operation) + "runtime work is executing; retry transition");
    case omni::physx::OmniPvdRecordingResult::eError:
    default:
        return set_error(OVPHYSX_API_ERROR, std::string(operation) + (discovery ? "failed to open the destination or start discovery" :
                                                            "failed to open the destination or start sampling"));
    }
}

OVPHYSX_API ovphysx_result_t ovphysx_ovd_stop_recording(ovphysx_instance_handle_t handle)
{
    OVPHYSX_API_PROFILE_ZONE(ovd_stop_recording);
    if (ovphysx::refuseBlockingReentry(nullptr) != OVPHYSX_API_SUCCESS)
        return {OVPHYSX_API_INVALID_STATE};
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "stop_recording: invalid handle");

    ovphysx::AllInstanceLanes everyLane;
    const ovphysx_api_status_t waitStatus = everyLane.acquire();
    if (waitStatus != OVPHYSX_API_SUCCESS)
        return set_error(waitStatus, "stop_recording: failed to reach a shared-runtime safe point");
    CompanionTransitionGuard companionTransition(companionRuntime());
    if (!companionTransition)
        return set_error(OVPHYSX_API_INVALID_STATE, "Companion runtime operation is executing; retry transition");
    std::unique_lock<std::mutex> recordingLock(g_gpuAttachMutex, std::defer_lock);
    if (enter_shared_runtime_safe_point(recordingLock) != OVPHYSX_API_SUCCESS)
        return set_error(OVPHYSX_API_INVALID_STATE, "stop_recording: failed to reach a shared-runtime safe point");
    omni::physx::IPhysxSimulation* physxSim =
        instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->stopOmniPvdRecording)
        return set_error(OVPHYSX_API_ERROR, "stop_recording: OmniPVD runtime API is unavailable");

    if (g_omniPvdRecordingOwner != handle)
        return set_error(OVPHYSX_API_INVALID_STATE, "stop_recording: no recording is active");
    bool recording = false;
    if (!physxSim->getOmniPvdRecordingStatus ||
        physxSim->getOmniPvdRecordingStatus(&recording) != omni::physx::RuntimeResult::eSuccess)
        return set_error(OVPHYSX_API_ERROR, "stop_recording: runtime status query failed");
    if (!recording)
        return set_error(OVPHYSX_API_INVALID_STATE, "stop_recording: no recording is active");

    const omni::physx::OmniPvdRecordingResult result = physxSim->stopOmniPvdRecording();
    reconcile_recording_owner_after_runtime_transition(physxSim);
    if (result == omni::physx::OmniPvdRecordingResult::eSuccess)
        return success();
    if (result == omni::physx::OmniPvdRecordingResult::eInvalidState)
        return set_error(OVPHYSX_API_INVALID_STATE, "stop_recording: no recording is active");
    if (result == omni::physx::OmniPvdRecordingResult::eNotSupported)
        return set_error(OVPHYSX_API_NOT_IMPLEMENTED, "stop_recording: OmniPVD recording is unsupported on this platform");
    return set_error(OVPHYSX_API_ERROR, "stop_recording: failed to finalize the recording");
}

OVPHYSX_API ovphysx_result_t ovphysx_ovd_is_recording(
    ovphysx_instance_handle_t handle, bool* out_is_recording)
{
    OVPHYSX_API_PROFILE_ZONE(ovd_is_recording);
    if (!out_is_recording)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_recording: out_is_recording is null");
    *out_is_recording = false;
    if (!get_instance(handle))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_recording: invalid handle");
    // Every transition that can end sampling clears the owner once the runtime reports it
    // ended, so the owner answers without waiting for instance work: the runtime's own
    // status query needs every instance idle.
    std::lock_guard<std::mutex> recordingLock(g_recordingMutex);
    *out_is_recording = g_omniPvdRecordingOwner == handle;
    return success();
}


// Kept as real exports for binaries built against the pre-migration 0.6 ABI.
OVPHYSX_API ovphysx_result_t ovphysx_start_recording(
    ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination)
{
    OVPHYSX_API_PROFILE_ZONE(start_recording);
    return ovphysx_ovd_start_recording(handle, destination, nullptr);
}

OVPHYSX_API ovphysx_result_t ovphysx_stop_recording(ovphysx_instance_handle_t handle)
{
    OVPHYSX_API_PROFILE_ZONE(stop_recording);
    return ovphysx_ovd_stop_recording(handle);
}

OVPHYSX_API ovphysx_result_t ovphysx_is_recording(ovphysx_instance_handle_t handle, bool* out_is_recording)
{
    OVPHYSX_API_PROFILE_ZONE(is_recording);
    return ovphysx_ovd_is_recording(handle, out_is_recording);
}

OVPHYSX_API ovphysx_result_t ovphysx_set_cpu_mode(bool cpu_only)
{
    OVPHYSX_API_PROFILE_ZONE(set_cpu_mode);
    return ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_CPU_ONLY,
        ovphysx_config_value_bool(cpu_only));
}

OVPHYSX_API ovphysx_result_t ovphysx_get_cpu_mode(bool* out_cpu_only)
{
    OVPHYSX_API_PROFILE_ZONE(get_cpu_mode);
    if (!out_cpu_only)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
            "ovphysx_get_cpu_mode: out_cpu_only must not be null");
    ovphysx_config_value_t value = {};
    const ovphysx_result_t result =
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, &value);
    if (result.status == OVPHYSX_API_SUCCESS)
        *out_cpu_only = value.value.bool_value;
    return result;
}

static constexpr char kShutdownCancelled[] = "Task cancelled: ovphysx_shutdown stopped the instance worker before the task started";

// Stops the worker of each live instance at shutdown, one instance at a time, with no
// other lock held: preempts its lane, which waits only for a running job, cancels every
// queued job and joins the worker. The instance stays registered for explicit destruction;
// its later submissions are rejected, and its receipts keep their outcomes until it is
// destroyed.
static void stop_live_instance_workers()
{
    std::vector<std::pair<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>> live;
    {
        std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
        for (const std::pair<const ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>& entry : g_instances)
        {
            if (entry.second && entry.second->executor)
                live.emplace_back(entry.first, entry.second);
        }
    }
    std::sort(live.begin(), live.end(), [](const auto& left, const auto& right) { return left.first < right.first; });
    for (const std::pair<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>& entry : live)
    {
        ovphysx::InstanceExecutor& executor = *entry.second->executor;
        ovphysx::InstanceExecutor::LaneHold lane = executor.preemptLane();
        executor.cancelQueued(kShutdownCancelled);
        executor.stopAndJoin();
    }
}

OVPHYSX_API ovphysx_result_t ovphysx_shutdown(void) {
    OVPHYSX_API_PROFILE_ZONE(shutdown);
    if (ovphysx::isInLogCallback())
        return set_error(OVPHYSX_API_ERROR, "ovphysx_shutdown cannot be called from within a log callback");
    // Shutdown joins every instance worker, so a callback, which may run under a held lane
    // or on a worker, and a worker thread itself are refused before anything changes.
    const ovphysx::Caller caller = ovphysx::currentCaller();
    if (caller.callback)
        return {ovphysx::refuseBlockingReentry(nullptr, caller)};

    // Companion leases must not be invalidated by process shutdown. Reserve
    // admission while checking so a new session cannot slip past this guard.
    std::unique_lock<std::shared_mutex> companionLock(g_instances_mutex);
    for (const std::pair<const uint64_t, std::shared_ptr<InstanceData>>& entry : g_instances)
    {
        if (entry.second->executor && entry.second->executor->isWorkerIdentity(caller.identity))
            return {ovphysx::refuseBlockingReentry(entry.second.get(), caller)};
    }
    if (companionRuntime().hasServiceLeases())
        return set_error(OVPHYSX_API_INVALID_STATE, "Runtime is busy: close companion service owners before shutdown");
    for (const std::pair<const uint64_t, std::shared_ptr<InstanceData>> & entry : g_instances)
        if (entry.second->companionLeases)
            return set_error(OVPHYSX_API_INVALID_STATE, "Runtime is busy: close companion sessions before shutdown");

    // Guard: must be called with a matching ovphysx_initialize().
    bool expected = true;
    if (!g_sdkLifecycle.initialized.compare_exchange_strong(expected, false, std::memory_order_acq_rel))
    {
        return set_error(OVPHYSX_API_ERROR, "ovphysx_shutdown called without matching ovphysx_initialize");
    }
    companionRuntime().setAdmissionOpen(false);
    companionLock.unlock();

    // Allow the next initialize() to re-sample OVPHYSX_DISABLE_GPU. Pre-init
    // get_cpu_mode() between shutdown and re-init reads the env live again.
    clearEnvGpuDisabledLatch();

    // With instances still alive, g_sdkLifecycle.initialized is cleared above so a second
    // shutdown() errors, but handles stay owned by their callers solely for
    // explicit destruction. Further work on them is unsupported. Their workers are
    // joined before callback delivery ends, so no worker thread runs once shutdown
    // returns, and unloading the library afterwards is safe. Successful shutdown still
    // disables and drains the application log callback.
    bool hasLiveInstances = false;
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        hasLiveInstances = !g_instances.empty();
    }
    if (hasLiveInstances)
    {
        stop_live_instance_workers();
        return ovphysx::shutdownLogCallback();
    }

    // Drain the direct PhysX runtime while Carbonite's settings/dictionary/log
    // plugins are still resident. omni::physx::runtime::shutdown() tears down
    // OmniPhysX (UJITSO processors, scenes, tensors, foundation runtime). Left to
    // C++ static destruction at process exit, those objects reach back into an
    // already-torn-down Carbonite and the process dies with an access violation,
    // preceded by "Leaked processor" UJITSO errors. The drain belongs on this
    // explicit, terminal call because the per-instance destroy path keeps the
    // runtime resident for device-switch create/destroy/create cycles.
    //
    // On Linux, stop Carbonite before removing its cache: its process-exit hook
    // otherwise calls _exit(), skipping earlier host callbacks and buffered streams.
    // Windows retains Carbonite's ExitProcess hook to protect dependency DLL teardown;
    // the packaged exit path bypasses release hooks and may leave private cache directories.
    removeExecutorWaitHooks();
    omni::physx::runtime::shutdown();
    ovphysx::shutdownCarbonite();
    {
        std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
        setRecordingOwner(OVPHYSX_INVALID_HANDLE);
    }

    g_runtimeConfigStarted.store(false, std::memory_order_release);

    return success();
}


// The time this step starts at, or false when advancing the internal counter by step_dt
// would overflow: even with finite inputs, current_time + step_dt can round up to +inf,
// which would then latch into the counter and poison every subsequent step.
static bool step_times(const InstanceData& instance, float step_dt, float& current_time, float& next_time)
{
    current_time = instance.sim_time;
    next_time = current_time + step_dt;
    return std::isfinite(next_time);
}

static constexpr char kStepOverflow[] = "sim_time would overflow (current_time + step_dt is not finite)";
static constexpr char kStepWithoutStage[] = "No stage attached";

// Fails job with the thread's last error, or a generic message when there is none.
static void fail_with_last_error(ovphysx::InstanceJob& job)
{
    const ThreadLocalError& error = tls_error();
    job.fail(error.last_error.empty() ? "Step execution failed" : error.last_error.c_str());
}

// The body of a step job, on the lane holder: earlier jobs have finished and no other call
// reads or writes the simulation state. submitted is the attachment the step was submitted
// for: a step without an attachment, or after an attach or detach whose failure was not
// collected, fails as a task and never reaches simulate().
static void run_step_job(ovphysx::InstanceJob& job, float step_dt, const ovphysx::SubmittedAttachment& submitted)
{
    InstanceData& instance = job.instance();
    if (!ovphysx::attachmentCurrent(instance, submitted))
    {
        job.fail(ovphysx::attachmentChangedCause(submitted.lifecycleTask));
        return;
    }
    if (instance.attachHandle == omni::physics::tensors::kNoAttach)
    {
        job.fail(kStepWithoutStage);
        return;
    }
    float current_time = 0.0f;
    float next_time = 0.0f;
    if (!step_times(instance, step_dt, current_time, next_time))
    {
        job.fail(kStepOverflow);
        return;
    }

    bool dispatched = false;
    ovphysx_api_status_t status = dispatch_simulation(instance, job.handle(), step_dt, current_time, dispatched);
    if (dispatched)
    {
        status = fetch_simulation(instance, status != OVPHYSX_API_SUCCESS);
        // Failed and zero-dt steps retain the last successful contact-force divisor.
        if (status == OVPHYSX_API_SUCCESS && step_dt > 0.0f)
            instance.last_step_dt = step_dt;
        // Time and warmup still advance after a dispatched step reports failure.
        instance.sim_time = next_time;
        // Tensor reads reuse this step's initialized simulation state.
        if (instance.attachHandle != omni::physics::tensors::kNoAttach)
        {
            instance.warmup_done.store(true, std::memory_order_release);
            instance.warmup_attach_handle.store(instance.attachHandle, std::memory_order_release);
        }
    }
    if (status != OVPHYSX_API_SUCCESS)
        fail_with_last_error(job);
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_step(ovphysx_instance_handle_t handle,
                                                          float step_dt) {
    OVPHYSX_API_PROFILE_ZONE(step);
    try {
        // Reject invalid dt before admission and sim-time updates.
        if (step_dt < 0.0f || !std::isfinite(step_dt))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");

        // The step is refused inside a callback, as every call that queues or waits for
        // instance work is.
        const ovphysx::Caller caller = ovphysx::currentCaller();
        if (caller.callback)
            return {ovphysx::refuseBlockingReentry(nullptr, caller), 0, 0};

        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor)
            return set_enqueue_error(OVPHYSX_API_ERROR, "Invalid handle");
        if (ovphysx::refuseBlockingReentry(instance.get(), caller) != OVPHYSX_API_SUCCESS)
            return {OVPHYSX_API_INVALID_STATE, 0, 0};

        process_pending_omnipvd_capture_request(handle);

        // The whole step, simulate() and fetchResults(), runs as one job after every earlier
        // job, so the call runs none of it: with no solver threads the runtime would run the
        // whole simulation inside simulate(). The job checks the attachment, which an attach
        // submitted earlier may not have made yet.
        const ovphysx::SubmittedAttachment submitted = ovphysx::submittedAttachment(*instance);
        std::unique_ptr<ovphysx::InstanceJob> job(new ovphysx::InstanceJob(
            "ovphysx_step", handle, instance,
            [step_dt, submitted](ovphysx::InstanceJob& self) { run_step_job(self, step_dt, submitted); }));
        return ovphysx::submitInstanceJob(*instance, handle, std::move(job));
    } catch (const std::exception& error) {
        return set_enqueue_error(OVPHYSX_API_ERROR, error.what());
    } catch (...) {
        return set_enqueue_error(OVPHYSX_API_ERROR, "Unknown exception during step");
    }
}

// Synchronous step+wait that does not create a public context. Equivalent to
// ovphysx_step() followed by wait_task(), but the step runs on the calling thread.
// Earlier tasks finish under the instance lane without collecting their outcomes; the
// synchronous step needs no task record.
OVPHYSX_API ovphysx_result_t ovphysx_step_sync(ovphysx_instance_handle_t handle,
                                                float step_dt) {
    OVPHYSX_API_PROFILE_ZONE(step_sync);
    if (step_dt < 0.0f || !std::isfinite(step_dt)) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");
    }

    process_pending_omnipvd_capture_request(handle);

    // The whole step runs under the instance lane: earlier work, queued step jobs
    // included, finishes first without being collected.
    ovphysx::InstanceLane lane;
    const ovphysx_api_status_t laneStatus = lane.acquire(handle);
    if (laneStatus == OVPHYSX_API_INVALID_STATE)
        return {laneStatus};
    if (laneStatus != OVPHYSX_API_SUCCESS)
        return set_error(laneStatus == OVPHYSX_API_NOT_FOUND ? OVPHYSX_API_ERROR : laneStatus,
                         "Failed to complete pending execution before synchronous step");

    const std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance) {
        return set_error(OVPHYSX_API_ERROR, "Invalid handle");
    }

    auto physxSim = instance->carbonite->getPhysxSimulation();
    const bool hasPhysicsStage = instance->attachHandle != omni::physics::tensors::kNoAttach;
    if (!physxSim || !hasPhysicsStage) {
        return set_error(OVPHYSX_API_ERROR, "No stage attached");
    }

    // Ensure attachStage() has been called after ovstage ingestion.
    // Must happen before physxSim->simulate().
    {
        ovphysx_api_status_t attach_status = ovphysx_ensure_physics_attached(handle);
        if (attach_status != OVPHYSX_API_SUCCESS)
            return set_error(attach_status, "Failed to attach physics stage");
    }

    // Read the internal counter as this step's start time, and guard the
    // advance against float overflow (current_time + step_dt rounding to +inf
    // would latch into the counter and poison subsequent steps).
    const float current_time = instance->sim_time;
    const float next_time = current_time + step_dt;
    if (!std::isfinite(next_time))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "sim_time would overflow (current_time + step_dt is not finite)");

    // Dispatch GPU work. Returns almost immediately.
    if (checkRuntimeResult(physxSim->simulate(instance->attachHandle, step_dt, current_time), "warmup simulate") != OVPHYSX_API_SUCCESS)
        return set_error(OVPHYSX_API_ERROR, "warmup: runtime rejected simulation");

    instance->warmup_done.store(true, std::memory_order_release);
    instance->warmup_attach_handle.store(instance->attachHandle, std::memory_order_release);

    // Blocks until the step results are ready.
    omni::physx::RuntimeErrorScope runtimeError;
    if (checkRuntimeResult(physxSim->fetchResults(instance->attachHandle), "warmup fetchResults") != OVPHYSX_API_SUCCESS)
        return set_error(OVPHYSX_API_ERROR, "warmup: runtime rejected fetchResults");
    if (runtimeError.message()[0])
        return set_error(OVPHYSX_API_ERROR, runtimeError.message());

    // ovphysx does not write results back to ovstage: state is read out via the
    // tensor-binding API and the application writes back to the Stage as needed.

    // Match ovphysx_step(): contact reads convert impulse to force with the dt
    // from the most recent successful step that simulated. A zero-dt step simulates
    // nothing and leaves the previous step's impulses in place, so it must not
    // overwrite the divisor.
    if (step_dt > 0.0f)
        instance->last_step_dt = step_dt;

    // Advance the internal counter after a successful step.
    instance->sim_time = next_time;

    return success();
}

// ========================================================================
// Typed config API
// ========================================================================

static ovphysx_result_t writeConfigValue(
    const OwnedConfigValue& source,
    ovphysx_config_value_t* outValue)
{
    if (!outValue)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Configuration output is null");

    outValue->type = source.type;
    switch (source.type)
    {
    case OVPHYSX_CONFIG_VALUE_BOOL:
        outValue->value.bool_value = source.boolValue;
        return success();
    case OVPHYSX_CONFIG_VALUE_INT64:
        outValue->value.int64_value = source.int64Value;
        return success();
    case OVPHYSX_CONFIG_VALUE_DOUBLE:
        outValue->value.double_value = source.doubleValue;
        return success();
    case OVPHYSX_CONFIG_VALUE_STRING:
    {
        const size_t capacity = outValue->value.string_value.length;
        const size_t required = source.stringValue.size() + 1;
        char* buffer = const_cast<char*>(outValue->value.string_value.ptr);
        if (!buffer || capacity == 0)
        {
            outValue->value.string_value.length = required;
            return set_error(OVPHYSX_API_BUFFER_TOO_SMALL, "Configuration string buffer is too small");
        }
        const size_t copied = std::min(source.stringValue.size(), capacity - 1);
        if (copied != 0)
            std::memcpy(buffer, source.stringValue.data(), copied);
        buffer[copied] = '\0';
        if (required > capacity)
        {
            outValue->value.string_value.length = required;
            return set_error(OVPHYSX_API_BUFFER_TOO_SMALL, "Configuration string buffer is too small");
        }
        outValue->value.string_value.length = source.stringValue.size();
        return success();
    }
    }
    return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Unknown configuration value type");
}

static bool profilerBackendFromEnvironment(int64_t& outValue)
{
    const char* value = std::getenv("OVPHYSX_PROFILER");
    if (!value || value[0] == '\0')
        return false;
    std::string normalized(value);
    std::transform(
        normalized.begin(),
        normalized.end(),
        normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (normalized == "none" || normalized == "off" || normalized == "0")
        outValue = OVPHYSX_PROFILER_BACKEND_NONE;
    else if (normalized == "nvtx")
        outValue = OVPHYSX_PROFILER_BACKEND_NVTX;
    else if (normalized == "trace")
        outValue = OVPHYSX_PROFILER_BACKEND_TRACE;
    else
        return false;
    return true;
}

OVPHYSX_API ovphysx_result_t ovphysx_set_global_config(
    ovphysx_global_config_key_t key,
    ovphysx_config_value_t value)
{
    OVPHYSX_API_PROFILE_ZONE(set_global_config);
    GlobalConfigSpec spec;
    if (!getGlobalConfigSpec(key, spec))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Unknown process-global configuration key");

    const char* validationError = nullptr;
    const ovphysx_api_status_t validation =
        validateGlobalConfigValue(key, spec, value, validationError);
    if (validation != OVPHYSX_API_SUCCESS)
        return set_error(validation, validationError ? validationError : "Invalid configuration value");

    // Create holds g_createInstanceMutex across its first log. A handler that
    // sets a startup-only key, including CPU_ONLY, must fail before this lock.
    if (spec.startupOnly && ovphysx::isInLogCallback())
        return set_error(
            OVPHYSX_API_ERROR,
            "Startup-only process-global configuration cannot be set from within a log callback");

    // Contact processing reaches every live attachment and the detached-runtime keys
    // depend on every instance's attachment: wait until each instance finished its
    // earlier work, then check and change under every lane. No such key is
    // startup-only, so the create lock below is never also taken.
    ovphysx::AllInstanceLanes everyLane;
    if (globalKeyNeedsEveryLane(key))
    {
        const ovphysx_api_status_t laneStatus = everyLane.acquire();
        if (laneStatus == OVPHYSX_API_INVALID_STATE)
            return {laneStatus};
        if (laneStatus != OVPHYSX_API_SUCCESS)
            return set_error(laneStatus, "Process-global configuration could not finish earlier instance work");
    }

    std::unique_lock<std::mutex> startupLock(g_createInstanceMutex, std::defer_lock);
    if (spec.startupOnly)
    {
        // Serialize the started check and retained write with instance creation.
        // Otherwise create could pass applyPendingGlobalConfig() between them.
        startupLock.lock();
        if (g_runtimeConfigStarted.load(std::memory_order_acquire))
            return set_error(
                OVPHYSX_API_INVALID_STATE,
                "Startup-only process-global configuration cannot change after instance creation");
    }

    if (globalKeyRequiresDetachedRuntime(key) && any_instance_attached())
    {
        return set_error(
            OVPHYSX_API_INVALID_STATE,
            "This process-global configuration cannot change while a stage is attached");
    }

    if (key == OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING)
    {
        const ovphysx_api_status_t status = updateLiveContactProcessing(value.value.bool_value);
        if (status != OVPHYSX_API_SUCCESS)
            return set_error(status, "Contact-processing configuration could not reach the attached stage");
    }

    if (key == OVPHYSX_GLOBAL_CONFIG_CPU_ONLY)
    {
        if (!value.value.bool_value && g_forceCpuMode.load(std::memory_order_acquire))
            return set_error(
                OVPHYSX_API_INVALID_STATE,
                "CPU-only configuration is sticky once enabled");
        g_forceCpuMode.store(value.value.bool_value, std::memory_order_release);
    }
    else if (key == OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL)
    {
        const ovphysx_result_t result =
            ovphysx::setLogLevel(static_cast<uint32_t>(value.value.int64_value));
        if (result.status != OVPHYSX_API_SUCCESS)
            return result;
    }
    else if (key == OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT)
    {
        const ovphysx_result_t result =
            ovphysx::setDefaultLogOutput(value.value.bool_value);
        if (result.status != OVPHYSX_API_SUCCESS)
            return result;
    }
    else
    {
        // CPU_ONLY, LOG_LEVEL, and DEFAULT_LOG_OUTPUT (the three keys above with
        // no settings path) have their own authoritative storage and never
        // consult g_globalConfigValues. The remaining keys are settings-backed,
        // except COMPILED_STAGE_CACHE: it has no settings path, so the cache
        // write below is its only storage, and attach reads it from there.
        const OwnedConfigValue owned = ownConfigValue(value);

        carb::Framework* framework = carb::getFramework();
        carb::settings::ISettings* settings =
            framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
        if (settings)
            applyOwnedConfigValue(settings, spec, owned);

        std::lock_guard<std::mutex> lock(g_scopedConfigMutex);
        g_globalConfigValues[static_cast<uint32_t>(key)] = owned;
    }
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config(
    ovphysx_global_config_key_t key,
    ovphysx_config_value_t* out_value)
{
    OVPHYSX_API_PROFILE_ZONE(get_global_config);
    if (!out_value)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Configuration output is null");

    GlobalConfigSpec spec;
    if (!getGlobalConfigSpec(key, spec))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Unknown process-global configuration key");

    OwnedConfigValue effective;
    effective.type = spec.type;
    if (key == OVPHYSX_GLOBAL_CONFIG_CPU_ONLY)
    {
        effective.boolValue = isProcessGpuDisabled();
        return writeConfigValue(effective, out_value);
    }
    if (key == OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL)
    {
        effective.int64Value = static_cast<int64_t>(ovphysx::getLogLevel());
        return writeConfigValue(effective, out_value);
    }
    if (key == OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT)
    {
        effective.boolValue = ovphysx::getDefaultLogOutput();
        return writeConfigValue(effective, out_value);
    }
    if (key == OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND ||
        key == OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH)
    {
        const ovphysx::BoundProfilerState bound = ovphysx::boundProfilerState();
        if (bound.bindFailed || bound.hostOwned)
        {
            if (key == OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND)
            {
                effective.int64Value = OVPHYSX_PROFILER_BACKEND_NONE;
                return writeConfigValue(effective, out_value);
            }
            return set_error(OVPHYSX_API_NOT_FOUND, "Configuration value is not set");
        }
        if (bound.active)
        {
            if (key == OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND)
            {
                effective.int64Value = bound.backend;
                return writeConfigValue(effective, out_value);
            }
            if (!bound.outputPathCaptured)
                return set_error(OVPHYSX_API_NOT_FOUND, "Configuration value is not set");
            effective.stringValue = bound.outputPath;
            return writeConfigValue(effective, out_value);
        }
    }
    if (key == OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND &&
        profilerBackendFromEnvironment(effective.int64Value))
    {
        return writeConfigValue(effective, out_value);
    }
    if (key == OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH)
    {
        const char* environmentPath = std::getenv("OVPHYSX_PROFILER_OUTPUT");
        if (environmentPath && environmentPath[0] != '\0')
        {
            effective.stringValue = environmentPath;
            return writeConfigValue(effective, out_value);
        }
    }

    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (settings && spec.path &&
        settings->getItemType(spec.path) != carb::dictionary::ItemType::eCount)
    {
        switch (spec.type)
        {
        case OVPHYSX_CONFIG_VALUE_BOOL:
            effective.boolValue = settings->getAsBool(spec.path);
            if (pathIsApproximateCollision(spec.path))
                effective.boolValue = !effective.boolValue;
            break;
        case OVPHYSX_CONFIG_VALUE_INT64:
            effective.int64Value = settings->getAsInt(spec.path);
            break;
        case OVPHYSX_CONFIG_VALUE_DOUBLE:
            effective.doubleValue = settings->getAsFloat(spec.path);
            break;
        case OVPHYSX_CONFIG_VALUE_STRING:
        {
            const char* stringValue = settings->getStringBuffer(spec.path);
            if (!stringValue)
                return set_error(OVPHYSX_API_NOT_FOUND, "Configuration value is not set");
            effective.stringValue = stringValue;
            break;
        }
        }
        return writeConfigValue(effective, out_value);
    }

    {
        std::lock_guard<std::mutex> lock(g_scopedConfigMutex);
        const std::unordered_map<uint32_t, OwnedConfigValue>::const_iterator found =
            g_globalConfigValues.find(static_cast<uint32_t>(key));
        if (found != g_globalConfigValues.end())
            return writeConfigValue(found->second, out_value);
    }

    OwnedConfigValue fallback;
    if (declaredGlobalDefault(key, fallback))
        return writeConfigValue(fallback, out_value);
    return set_error(OVPHYSX_API_NOT_FOUND, "Configuration value is not set");
}

OVPHYSX_API ovphysx_result_t ovphysx_set_config(
    ovphysx_instance_handle_t instance,
    ovphysx_config_key_t key,
    ovphysx_config_value_t value)
{
    OVPHYSX_API_PROFILE_ZONE(set_config);
    if (key != OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Unknown instance configuration key");
    if (value.type != OVPHYSX_CONFIG_VALUE_STRING)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Instance configuration value type does not match key");
    if ((!value.value.string_value.ptr && value.value.string_value.length != 0) ||
        configStringHasEmbeddedNull(value.value.string_value))
    {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "active_cuda_gpus is not a valid string");
    }

    // The attachment check and the write run under the instance lane, after earlier
    // work on the instance.
    ovphysx::InstanceLane lane;
    const ovphysx_api_status_t laneStatus = lane.acquire(instance);
    if (laneStatus == OVPHYSX_API_INVALID_STATE)
        return {laneStatus};

    std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
    InstanceData* instanceData = get_instance_ptr(instance);
    if (!instanceData)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid instance handle");
    if (instanceData->attachHandle != omni::physics::tensors::kNoAttach)
        return set_error(
            OVPHYSX_API_INVALID_STATE,
            "Instance configuration cannot change after stage attachment");

    const char* stringData = value.value.string_value.ptr;
    const size_t stringLength = value.value.string_value.length;
    char parseError[256] = {};
    std::vector<int32_t> ordinals =
        parseActiveCudaGpus(stringData, stringLength, parseError, sizeof(parseError));
    if (parseError[0] != '\0' || ordinals.empty())
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT,
            parseError[0] != '\0' ? parseError : "active_cuda_gpus could not be parsed");

    if (stringLength != 0 && !isProcessGpuDisabled() && omni::physx::cudaShim::isCudaAvailable())
    {
        char validationErr[384] = {};
        if (!validateActiveCudaGpuOrdinals(ordinals, validationErr, sizeof(validationErr)))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, validationErr);
    }

    instanceData->active_cuda_gpus.assign(stringData ? stringData : "", stringLength);
    instanceData->active_cuda_gpus_configured = true;
    if (stringLength == 0)
        instanceData->active_cuda_ordinals.clear();
    else
        instanceData->active_cuda_ordinals = std::move(ordinals);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_config(
    ovphysx_instance_handle_t instance,
    ovphysx_config_key_t key,
    ovphysx_config_value_t* out_value)
{
    OVPHYSX_API_PROFILE_ZONE(get_config);
    if (!out_value)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Configuration output is null");
    if (key != OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Unknown instance configuration key");

    std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
    InstanceData* instanceData = get_instance_ptr(instance);
    if (!instanceData)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid instance handle");
    if (!instanceData->active_cuda_gpus_configured)
        return set_error(OVPHYSX_API_NOT_FOUND, "Instance configuration value is not set");

    OwnedConfigValue effective;
    effective.type = OVPHYSX_CONFIG_VALUE_STRING;
    effective.stringValue = instanceData->active_cuda_gpus;
    return writeConfigValue(effective, out_value);
}

OVPHYSX_API ovphysx_result_t ovphysx_set_global_config_entry(ovphysx_config_entry_t entry) {
    OVPHYSX_API_PROFILE_ZONE(set_global_config_entry);
    if (isRetiredOvdConfigEntry(entry))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, kRetiredOvdConfigError);
    std::unique_lock<std::mutex> createLock(g_createInstanceMutex, std::defer_lock);
    ovphysx_global_config_key_t overlapKey = {};
    GlobalConfigSpec overlapSpec;
    const bool overlapStartup =
        legacyEntryMapsToGlobalKey(entry, overlapKey) &&
        getGlobalConfigSpec(overlapKey, overlapSpec) &&
        overlapSpec.startupOnly;
    if (overlapStartup && ovphysx::isInLogCallback())
        return set_error(
            OVPHYSX_API_ERROR,
            "Startup-only process-global configuration cannot be set from within a log callback");
    const ovphysx_api_status_t workerStatus = validateWorkerPoolStartupEntry(entry);
    if (workerStatus != OVPHYSX_API_SUCCESS)
        return set_error(workerStatus, "Invalid worker pool startup config");
    // Entries that reach live attachments or need a detached runtime take every
    // instance's lane, as ovphysx_set_global_config() does. They are neither
    // startup-only nor worker-pool entries, so the create lock below stays untaken.
    ovphysx::AllInstanceLanes everyLane;
    if (entryNeedsEveryLane(entry))
    {
        const ovphysx_api_status_t laneStatus = everyLane.acquire();
        if (laneStatus == OVPHYSX_API_INVALID_STATE)
            return {laneStatus};
        if (laneStatus != OVPHYSX_API_SUCCESS)
            return set_error(laneStatus, "Process-global configuration could not finish earlier instance work");
    }
    // Hold g_createInstanceMutex across the started-pool check and apply.
    // Otherwise a concurrent ovphysx_create_instance() can start tasking
    // between the two and this call reports success for a write that cannot
    // resize the pool.
    if (isWorkerPoolStartupEntry(entry))
    {
        createLock.lock();
        if (isTaskingStarted())
            return set_error(
                OVPHYSX_API_ERROR,
                "Worker pool startup config cannot change after carb.tasking has started");
    }
    if (overlapStartup && !createLock.owns_lock())
        createLock.lock();
    const auto acquireSettings = []() -> carb::settings::ISettings*
    {
        carb::Framework* framework = carb::getFramework();
        return framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    };
    carb::settings::ISettings* settings = acquireSettings();
    const char* entryError = nullptr;
    if (!settings)
    {
        // Carbonite starts with the first instance. Create holds this lock
        // through its first log, so a log callback cannot wait for it.
        if (!createLock.owns_lock())
        {
            if (ovphysx::isInLogCallback())
                return set_error(OVPHYSX_API_ERROR,
                                 "Config entries cannot be queued from within a log callback");
            createLock.lock();
            settings = acquireSettings();
        }
        if (!settings)
        {
            const ovphysx_api_status_t queued = queueConfigEntry(entry, entryError);
            if (queued != OVPHYSX_API_SUCCESS)
                return set_error(queued, entryError ? entryError : "Invalid config entry");
            return {OVPHYSX_API_SUCCESS};
        }
    }
    ovphysx_api_status_t status = applyConfigEntry(settings, entry, false, entryError);
    if (status != OVPHYSX_API_SUCCESS)
        return set_error(status, entryError ? entryError : "Invalid config entry");
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_test_get_settings_bool(const char* path, bool* out_value)
{
    if (!path || !out_value)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings)
        return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    if (settings->getItemType(path) == carb::dictionary::ItemType::eCount)
        return set_error(OVPHYSX_API_NOT_FOUND, "Configuration value is not set");
    *out_value = settings->getAsBool(path);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_bool(ovphysx_config_bool_t key, bool* out_value) {
    OVPHYSX_API_PROFILE_ZONE(get_global_config_bool);
    if (!out_value || key < 0 || key >= OVPHYSX_CONFIG_BOOL_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    if (!s_boolKeyPaths[key])
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, kRetiredOvdConfigError);
    ovphysx_config_entry_t probe = {};
    probe.key_type = OVPHYSX_CONFIG_KEY_TYPE_BOOL;
    probe.key.bool_key = key;
    ovphysx_global_config_key_t globalKey;
    if (legacyEntryMapsToGlobalKey(probe, globalKey))
    {
        ovphysx_config_value_t value = {};
        const ovphysx_result_t result = ovphysx_get_global_config(globalKey, &value);
        if (result.status != OVPHYSX_API_SUCCESS)
            return result;
        *out_value = value.value.bool_value;
        return success();
    }
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    *out_value = settings->getAsBool(s_boolKeyPaths[key]);
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_int32(ovphysx_config_int32_t key, int32_t* out_value) {
    OVPHYSX_API_PROFILE_ZONE(get_global_config_int32);
    if (!out_value || key < 0 || key >= OVPHYSX_CONFIG_INT32_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    if (!s_int32KeyPaths[key])
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, kRetiredOvdConfigError);
    ovphysx_config_entry_t probe = {};
    probe.key_type = OVPHYSX_CONFIG_KEY_TYPE_INT32;
    probe.key.int32_key = key;
    ovphysx_global_config_key_t globalKey;
    if (legacyEntryMapsToGlobalKey(probe, globalKey))
    {
        ovphysx_config_value_t value = {};
        const ovphysx_result_t result = ovphysx_get_global_config(globalKey, &value);
        if (result.status != OVPHYSX_API_SUCCESS)
            return result;
        *out_value = static_cast<int32_t>(value.value.int64_value);
        return success();
    }
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    *out_value = settings->getAsInt(s_int32KeyPaths[key]);
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_float(ovphysx_config_float_t key, float* out_value) {
    OVPHYSX_API_PROFILE_ZONE(get_global_config_float);
    if (!out_value || key < 0 || key >= OVPHYSX_CONFIG_FLOAT_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    *out_value = settings->getAsFloat(s_floatKeyPaths[key]);
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_string(ovphysx_config_string_t key, ovphysx_string_t* value_out, size_t* out_required_size) {
    OVPHYSX_API_PROFILE_ZONE(get_global_config_string);
    if (!value_out || !value_out->ptr || value_out->length == 0 || !out_required_size ||
        key < 0 || key >= OVPHYSX_CONFIG_STRING_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    if (!s_stringKeyPaths[key])
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, kRetiredOvdConfigError);
    if (value_out->length > UINT32_MAX)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Buffer capacity exceeds UINT32_MAX");
    const size_t buffer_capacity = value_out->length;
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    char* mutable_buffer = const_cast<char*>(value_out->ptr);
    size_t required_size = 0;
    if (!getSettingValueAsString(
            settings,
            s_stringKeyPaths[key],
            mutable_buffer,
            static_cast<uint32_t>(buffer_capacity),
            &required_size)) {
        *out_required_size = 0;
        return set_error(OVPHYSX_API_NOT_FOUND, "Config value not found");
    }
    *out_required_size = required_size;
    if (required_size > buffer_capacity)
    {
        return set_error(OVPHYSX_API_BUFFER_TOO_SMALL, "Buffer too small");
    }
    value_out->length = required_size - 1;
    return {OVPHYSX_API_SUCCESS};
}

namespace
{
// The inputs of one ovphysx_attach_ovstage() call, copied at submission. The Stage itself
// stays borrowed until the task completes.
struct AttachRequest
{
    ovstage_instance_t* stage = nullptr;
    ovstage_ordinal_t readOrdinal = 0;
    // /ovphysx/clone/useEnvIds, read when the call is made.
    bool useEnvIds = true;
    // OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE as configured when the call is made, and
    // the cache mode it resolved to then.
    int64_t configuredCacheMode = OVPHYSX_COMPILED_STAGE_CACHE_AUTOMATIC;
    omni::physx::PhysxCompiledStageCacheMode cacheMode = omni::physx::PhysxCompiledStageCacheMode::eDisabled;
    // The CUDA context current on the submitting thread, which queued execution
    // reproduces on a worker or waiter; zero when none was current. Calling-thread
    // execution already has the submitting context and does not apply this snapshot.
    uintptr_t cudaContext = 0;
};

constexpr char kCompanionTransitionBusy[] = "Companion runtime operation is executing; retry transition";
} // namespace

// Whether an attach or detach submitted now runs on the calling thread instead of as a
// job: while a recording is active or starting. Recording start and stop take every
// instance's lane, so a lifecycle job queued before a start finishes before the recording
// starts, and no lifecycle job ever runs while one is active: finalizing a recording needs
// every lane, which a job holding its own lane must never wait for. Call with
// g_recordingMutex held, and queue the job before releasing it.
static bool lifecycle_runs_on_caller()
{
    return g_omniPvdRecordingOwner != OVPHYSX_INVALID_HANDLE || g_recordingStartsPending != 0;
}

// The attach of every venue, under the lane(s) and the lifecycle lock its caller holds:
// a queued attach job on the worker or a waiter (queuedAttach), or calling-thread
// execution for a workerless instance or recording. Earlier work has run, so the
// checks see the state it left behind. Returns SUCCESS, or ERROR with the task
// failure text in failure.
static ovphysx_api_status_t attach_ovstage_body(ovphysx_instance_handle_t handle, InstanceData& instance,
                                                const AttachRequest& request, bool queuedAttach, std::string& failure)
{
    if (instance.attachHandle != omni::physics::tensors::kNoAttach)
    {
        failure = "attach_ovstage: a stage is already attached; detach before attaching ovstage";
        return OVPHYSX_API_ERROR;
    }
    omni::physx::IPhysxSimulation* physxSim =
        instance.carbonite ? instance.carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim)
    {
        failure = "attach_ovstage: IPhysxSimulation is unavailable";
        return OVPHYSX_API_ERROR;
    }

    instance.ovstage_attach_payload.instance = request.stage;
    instance.ovstage_attach_payload.dict = nullptr;
    // Production attaches consume ovstage alone; no USD stage-cache identity is needed.
    instance.ovstage_attach_payload.usdStageId = 0;

    // Creation-time replicator env-ids: have the runtime assign environment ids at parse
    // (GPU dynamics + broadphase only) so a later clone() finds the source collision-isolated
    // from co-located copies. Ids can only be set on objects outside a scene, so this must run
    // before the parse. Follows /ovphysx/clone/useEnvIds (default on) as read at submission;
    // the lifecycle lock keeps another attach from changing the process-global setting.
    if (carb::Framework* fw = carb::getFramework()) {
        if (carb::settings::ISettings* settings = fw->tryAcquireInterface<carb::settings::ISettings>())
            settings->setBool(omni::physx::kSettingReplicatorEnvIdsOnAttach, request.useEnvIds);
    }

    // The runtime copies the cache mode for this attachment's later insertions and
    // writes the result before returning; it retains neither pointer.
    omni::physx::PhysxCompiledStageCacheResult cacheResult = omni::physx::PhysxCompiledStageCacheResult::eDisabled;
    omni::physx::PhysxCompiledStageCacheOptions cacheOptions;
    cacheOptions.mode = request.cacheMode;
    cacheOptions.result = &cacheResult;
    // A host's framework without cooking or a cache directory leaves the cache no
    // datastore of ovphysx's own: an enabled mode reports that instead of caching.
    const bool cacheUnavailable = request.cacheMode != omni::physx::PhysxCompiledStageCacheMode::eDisabled &&
                                  ovphysx::compiledStageCacheUnavailable();
    if (cacheUnavailable)
        cacheOptions.mode = omni::physx::PhysxCompiledStageCacheMode::eDisabled;
    omni::physx::PhysxAttachOptions attachOptions;
    attachOptions.compiledStageCache = &cacheOptions;

    // GPU selection is process-global and PhysX consumes it synchronously while
    // attachOvstage creates the first GPU scene, so the setting and the attach
    // stay in one transaction. The caller owns the sealed read ordinal, and the
    // initial scene parse reads at it. On the worker the submitting thread's CUDA
    // context is current meanwhile: active-context attach creates the scene in it.
    omni::physx::RuntimeErrorScope runtimeError;
    omni::physics::AttachHandle attachHandle = omni::physics::kNoAttach;
    bool busy = false;
    {
        std::lock_guard<std::mutex> attachLock(g_gpuAttachMutex);
        WorkerCudaContextScope cudaContext(request.cudaContext, queuedAttach);
        if (cudaContext.failed())
        {
            instance.ovstage_attach_payload = OvstageAttachPayload{};
            failure = "attach_ovstage: the submitting CUDA context state could not be reproduced "
                      "on the executing thread (CUDA error " + std::to_string(cudaContext.status()) + ")";
            return OVPHYSX_API_ERROR;
        }
        applyAttachTimeGpuSelection(instance, carb::getFramework(), "attachOvstage");
        attachHandle = omni::physx::runtime::attachOvstageWithAdmission(
            &instance.ovstage_attach_payload, request.readOrdinal, attachOptions, busy);
    }
    if (cacheUnavailable)
        cacheResult = omni::physx::PhysxCompiledStageCacheResult::eUnavailable;
    // A busy runtime refused the attach before it reached the cache.
    if (!busy) {
        CARB_LOG_INFO("[ovphysx] attach_ovstage: compiled stage cache: %s (mode %s%s)",
                      compiledStageCacheResultName(cacheResult), compiledStageCacheModeName(request.cacheMode),
                      request.configuredCacheMode == OVPHYSX_COMPILED_STAGE_CACHE_AUTOMATIC ? ", automatic" : "");
    }
    if (attachHandle == omni::physics::kNoAttach) {
        instance.ovstage_attach_payload = OvstageAttachPayload{};
        if (busy)
        {
            failure = "attach_ovstage: the runtime was running another operation; submit the attach again";
            return OVPHYSX_API_ERROR;
        }
        static constexpr char kContext[] = "attach_ovstage: runtime attachment failed";
        failure = runtimeError.message()[0] ? std::string(kContext) + ": " + runtimeError.message()
                                            : std::string(kContext);
        return OVPHYSX_API_ERROR;
    }

    // Two different things, deliberately read separately (ADR-0013): the backing
    // USD stage id, used only for USD-stage lifecycle below, and the attach handle,
    // which identifies this attach and is nonzero even when there is no stage.
    const int64_t stageId = physxSim->getAttachedStage
        ? static_cast<int64_t>(physxSim->getAttachedStage(attachHandle))
        : 0;

    // Scans of live attachments read these fields under g_instances_mutex.
    // Publish them after g_gpuAttachMutex is released so a concurrent config
    // or recording check cannot observe "nothing attached" once attachOvstage
    // has succeeded. Lock order is g_gpuAttachMutex, then g_instances_mutex;
    // never take the GPU attach mutex while holding the instances mutex.
    {
        std::unique_lock<std::shared_mutex> liveLock(g_instances_mutex);
        instance.attachedStageId = stageId;
        instance.attachHandle = attachHandle;
        instance.visualizationScopeDropped = false;
        instance.ovstage_attached = true;
    }
    omni::physx::ovxDebugRenderNoteAttach(handle, attachHandle, 0);
    if (!ovphysx_object_change_attach_instance(handle, attachHandle))
    {
        // The rollback runs under the lanes and the lifecycle lock this attach holds and
        // never takes them again. Like a detach it needs companion admission; while a
        // companion operation runs, the attachment stays and the task still fails.
        if (queuedAttach)
        {
            CompanionJobTransitionGuard companion(companionRuntime());
            if (companion)
                detach_ovstage_body(handle, instance);
        }
        else
        {
            CompanionTransitionGuard companion(companionRuntime());
            if (companion)
                detach_ovstage_body(handle, instance);
        }
        failure = "attach_ovstage: failed to bind object-change subscriptions";
        return OVPHYSX_API_ERROR;
    }
    // Physics is attached whenever the attach succeeded. A zero stage id only
    // means there is no backing USD stage.
    instance.resetStageFlags(attachHandle != omni::physics::tensors::kNoAttach);
    if (stageId != 0) {
        // Genuinely about the USD stage object, so it stays keyed by stage id.
        registerStageLifecycleEntry(stageId);
    }
    return OVPHYSX_API_SUCCESS;
}

// The body of an attach job, run by the lane holder. generation is
// the attachment generation the attach took at submission.
static void run_attach_job(ovphysx::InstanceJob& job, const AttachRequest& request, uint64_t generation)
{
    std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
    std::string failure;
    const bool queuedAttach = job.instance().executor && job.instance().executor->hasWorker();
    if (attach_ovstage_body(job.handle(), job.instance(), request, queuedAttach, failure) != OVPHYSX_API_SUCCESS)
    {
        job.fail(failure);
        return;
    }
    job.instance().executed_generation.store(generation, std::memory_order_release);
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_attach_ovstage(ovphysx_instance_handle_t handle,
                                                     ovstage_instance_t* stage,
                                                     ovstage_ordinal_t read_ordinal) {
    OVPHYSX_API_PROFILE_ZONE(attach_ovstage);
    TaskContext admission;
    try {
        if (!stage) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "attach_ovstage: stage is null");
        }
        // 0 is AttachedStage's internal sentinel for "use the payload attach-time
        // ordinal". Storing it as the skip cursor leaves replay of the real attach
        // ordinal unguarded. The public contract is a caller-owned sealed ordinal.
        // Samples and the Python default start at 1. The runtime's ordinals are
        // unsigned, so the signed ovstage ordinals below 1 are not attachable.
        if (read_ordinal <= 0) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "attach_ovstage: read_ordinal must be a positive caller-owned sealed ordinal; 0 is reserved");
        }

        // The attach is refused inside a callback, as every call that queues or waits for
        // instance work is.
        const ovphysx::Caller caller = ovphysx::currentCaller();
        if (caller.callback)
            return {ovphysx::refuseBlockingReentry(nullptr, caller), 0, 0};
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "attach_ovstage: invalid handle");
        }
        if (ovphysx::refuseBlockingReentry(instance.get(), caller) != OVPHYSX_API_SUCCESS)
            return {OVPHYSX_API_INVALID_STATE, 0, 0};

        omni::physx::IPhysxSimulation* physxSim =
            instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
        if (!physxSim) {
            return set_enqueue_error(OVPHYSX_API_ERROR,
                             "attach_ovstage: IPhysxSimulation is unavailable");
        }

        // Verify before issuing a receipt or touching attachment state. A missed registration
        // changes the populated physics configuration and cannot be repaired in this process.
        std::string schemaError;
        if (!ovphysx::schema_registration::verifyPhysxSchemasRegistered(schemaError)) {
            bool requireRegistration = true;
            if (carb::Framework* framework = carb::getFramework()) {
                if (carb::settings::ISettings* settings =
                        framework->tryAcquireInterface<carb::settings::ISettings>()) {
                    settings->setDefaultBool(ovphysx::schema_registration::kRequireRegistration, true);
                    requireRegistration = settings->getAsBool(ovphysx::schema_registration::kRequireRegistration);
                }
            }
            if (requireRegistration) {
                return set_enqueue_error(OVPHYSX_API_ERROR, schemaError);
            }
            CARB_LOG_WARN("[ovphysx] %s (continuing: %s is false)", schemaError.c_str(),
                          ovphysx::schema_registration::kRequireRegistration);
        }

        // Reserve companion transition admission before copying inputs or observing
        // attach-time settings. A later companion operation is rejected until the
        // lifecycle work finishes, so accepted work cannot fail solely because the
        // worker lost a timing race with that operation.
        std::shared_ptr<CompanionJobTransitionGuard> companionJobTransition =
            std::make_shared<CompanionJobTransitionGuard>(companionRuntime());
        if (!*companionJobTransition)
            return set_enqueue_error(OVPHYSX_API_INVALID_STATE, kCompanionTransitionBusy);

        // Inputs are copied now: the job runs after the call returns. The checks that depend
        // on earlier work (already attached) and the runtime attach belong to the job.
        std::shared_ptr<AttachRequest> request = std::make_shared<AttachRequest>();
        request->stage = stage;
        request->readOrdinal = read_ordinal;
        if (carb::Framework* fw = carb::getFramework()) {
            if (carb::settings::ISettings* settings = fw->tryAcquireInterface<carb::settings::ISettings>()) {
                settings->setDefaultBool("/ovphysx/clone/useEnvIds", true);
                request->useEnvIds = settings->getAsBool("/ovphysx/clone/useEnvIds");
            }
        }
        // The cache mode is runtime mutable; this attach uses the mode configured now.
        request->configuredCacheMode = configuredCompiledStageCacheMode();
        request->cacheMode = resolveCompiledStageCacheMode(request->configuredCacheMode);
        request->cudaContext = submittingCudaContext();
        // The attachment generation this attach takes once it is accepted (see InstanceData).
        const uint64_t generation = instance->lifecycle_generations + 1;
        std::unique_ptr<ovphysx::InstanceJob> job(new ovphysx::InstanceJob(
            "ovphysx_attach_ovstage", handle, instance,
            [request, generation, companionJobTransition](ovphysx::InstanceJob& self) mutable {
                struct ReleaseTransition
                {
                    std::shared_ptr<CompanionJobTransitionGuard>& value;
                    ~ReleaseTransition() { value.reset(); }
                } release{ companionJobTransition };
                run_attach_job(self, *request, generation);
            }));
        companionJobTransition.reset(); // The job now owns the reservation.
        {
            std::unique_lock<std::mutex> recordingLock(g_recordingMutex);
            if (!lifecycle_runs_on_caller())
            {
                const ovphysx_enqueue_result_t receipt = ovphysx::queueInstanceJob(*instance, handle, std::move(job));
                if (receipt.status == OVPHYSX_API_SUCCESS)
                    ovphysx::noteLifecycleSubmission(*instance, generation, receipt.op_index);
                recordingLock.unlock();
                return ovphysx::runWithoutWorker(*instance, receipt);
            }
        }

        // While a recording is active the attach runs here, after every instance's earlier
        // work, under every lane and then the lifecycle lock: a failed attach detaches
        // again, and that detach may finalize the recording. The calling thread's CUDA
        // context is current already.
        request->cudaContext = 0;
        ovphysx::AllInstanceLanes everyLane;
        ovphysx_api_status_t wait_status = everyLane.acquire();
        if (wait_status == OVPHYSX_API_INVALID_STATE) {
            return {wait_status, 0, 0};
        }
        if (wait_status == OVPHYSX_API_SUCCESS && get_instance(handle) != instance) {
            wait_status = OVPHYSX_API_NOT_FOUND;
        }
        if (wait_status != OVPHYSX_API_SUCCESS) {
            return set_enqueue_error(wait_status, "Failed to complete pending operations before attach_ovstage");
        }
        std::string failure;
        ovphysx_api_status_t attachStatus = OVPHYSX_API_ERROR;
        {
            std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
            admission.create_task(handle);
            ovphysx::noteLifecycleSubmission(*instance, generation, admission.receipt.op_index);
            attachStatus = attach_ovstage_body(handle, *instance, *request, /*queuedAttach=*/false, failure);
            if (attachStatus == OVPHYSX_API_SUCCESS)
                instance->executed_generation.store(generation, std::memory_order_release);
        }
        // The direct path does not run the job object; destroying it releases the
        // submission reservation before publishing the task outcome below.
        job.reset();
        if (attachStatus != OVPHYSX_API_SUCCESS)
            return admission.fail(failure.c_str());
        return admission.complete();
    } catch (const std::exception& error) {
        return admission.fail(error.what());
    } catch (...) {
        return admission.fail("Unknown exception during attach_ovstage");
    }
}

// The body of an update job on the lane holder: earlier jobs have finished, so the
// attachment checks see the state they left behind.
static void run_update_job(ovphysx::InstanceJob& job, omni::physx::IPhysxSimulation* physxSim, uint64_t from_ordinal,
                           uint64_t to_ordinal, const ovphysx::SubmittedAttachment& submitted)
{
    InstanceData& instance = job.instance();
    if (!ovphysx::attachmentCurrent(instance, submitted))
    {
        job.fail(ovphysx::attachmentChangedCause(submitted.lifecycleTask));
        return;
    }
    if (!instance.ovstage_attached)
    {
        job.fail("update_from_ovstage: no ovstage is attached");
        return;
    }

    omni::physx::RuntimeErrorScope runtimeError;
    if (!physxSim->updateFromOvStage(instance.attachHandle, from_ordinal, to_ordinal))
    {
        static constexpr char kContext[] = "update_from_ovstage: IPhysxSimulation::updateFromOvStage failed";
        job.fail(runtimeError.message()[0] ? std::string(kContext) + ": " + runtimeError.message()
                                           : std::string(kContext));
    }
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_from_ovstage(ovphysx_instance_handle_t handle,
                                                        ovstage_ordinal_range_t range) {
    OVPHYSX_API_PROFILE_ZONE(update_from_ovstage);
    try {
        // ovstage's range: [start_ordinal, end_ordinal] when has_start_ordinal, else the
        // single end_ordinal. The runtime drain takes a closed [from, to].
        const ovstage_ordinal_t start = range.has_start_ordinal ? range.start_ordinal : range.end_ordinal;
        if (start > range.end_ordinal) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "update_from_ovstage: range.start_ordinal must be <= range.end_ordinal");
        }
        // The runtime's ordinals are unsigned and the attach ordinal is at least 1, so every
        // ordinal below 1 is already consumed: clamping into [1, OVSTAGE_ORDINAL_MAX] keeps the
        // range's meaning, and a range wholly below 1 stays a successful no-op.
        const uint64_t from_ordinal = static_cast<uint64_t>(std::max<ovstage_ordinal_t>(start, 1));
        const uint64_t to_ordinal = static_cast<uint64_t>(std::max<ovstage_ordinal_t>(range.end_ordinal, 1));

        // The update is refused inside a callback, as every call that queues or waits for
        // instance work is.
        const ovphysx::Caller caller = ovphysx::currentCaller();
        if (caller.callback)
            return {ovphysx::refuseBlockingReentry(nullptr, caller), 0, 0};
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "update_from_ovstage: invalid handle");
        }
        if (ovphysx::refuseBlockingReentry(instance.get(), caller) != OVPHYSX_API_SUCCESS)
            return {OVPHYSX_API_INVALID_STATE, 0, 0};
        // A graph replay does not repeat this host-side update, so a capture would keep only the run made
        // while capturing. Only a capture of a blocking stream in the current context is seen, whichever
        // thread runs it.
        if (legacyStreamHeldByCapture()) {
            return set_enqueue_error(OVPHYSX_API_INVALID_STATE,
                             "update_from_ovstage: refused while a CUDA graph capture of a blocking stream is in "
                             "progress in this CUDA context; a graph replay would not repeat this update");
        }

        omni::physx::IPhysxSimulation* physxSim =
            instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
        if (!physxSim || !physxSim->updateFromOvStage) {
            return set_enqueue_error(OVPHYSX_API_ERROR,
                             "update_from_ovstage: IPhysxSimulation::updateFromOvStage is unavailable");
        }

        // The update runs as one job on the instance worker after every earlier job. Its
        // attachment check is part of the job, so an attach submitted earlier counts.
        const ovphysx::SubmittedAttachment submitted = ovphysx::submittedAttachment(*instance);
        std::unique_ptr<ovphysx::InstanceJob> job(new ovphysx::InstanceJob(
            "ovphysx_update_from_ovstage", handle, instance,
            [physxSim, from_ordinal, to_ordinal, submitted](ovphysx::InstanceJob& self) {
                run_update_job(self, physxSim, from_ordinal, to_ordinal, submitted);
            }));
        return ovphysx::submitInstanceJob(*instance, handle, std::move(job));
    } catch (const std::exception& error) {
        return set_enqueue_error(OVPHYSX_API_ERROR, error.what());
    } catch (...) {
        return set_enqueue_error(OVPHYSX_API_ERROR, "Unknown exception during update_from_ovstage");
    }
}

namespace
{
// The inputs of one ovphysx_clone() call, copied at submission: the caller may free or
// overwrite its strings and arrays as soon as the call returns. The clone job reads them
// on the instance worker.
struct CloneRequest
{
    std::string source;
    // The target paths in call order, their C strings for the runtime entry, and the same
    // paths as a set, which rejects duplicates in the call at submission and is merged into
    // the attachment's cloned targets when the clone succeeds.
    std::vector<std::string> targets;
    std::vector<const char*> targetPointers;
    std::unordered_set<std::string> targetSet;
    // num_targets * 7 floats, or empty to co-locate every copy on the source.
    std::vector<float> anchorTransforms;
    // num_targets logical environment ids, or empty for per-call numbering.
    std::vector<uint32_t> envIds;
    bool useEnvIds = true;
};
} // namespace

// The body of a clone job on the lane holder: earlier jobs have finished, so the checks
// below see the attachment, warmup and step state they left behind.
static void run_clone_job(ovphysx::InstanceJob& job, omni::physx::IPhysxSimulation* physxSim, CloneRequest& request,
                          const ovphysx::SubmittedAttachment& submitted)
{
    InstanceData& instance = job.instance();
    if (!ovphysx::attachmentCurrent(instance, submitted))
    {
        job.fail(ovphysx::attachmentChangedCause(submitted.lifecycleTask));
        return;
    }
    // ovphysx attaches only via ovstage, so cloning runs on the ovstage attach. (The underlying
    // runtime seam is attach-agnostic, but that path is not exposed here.)
    if (!instance.ovstage_attached)
    {
        job.fail("clone: no ovstage is attached (call ovphysx_attach_ovstage first)");
        return;
    }

    // clone() must be called before warmup() or the first step() in all modes.
    // On GPU, warmup allocates DirectGPU buffers sized by actor count, and cloning
    // after that reallocates and corrupts simulation state. CPU enforces the same
    // ordering so code validated in CPU mode behaves the same on GPU.
    if (instance.warmup_done.load(std::memory_order_acquire) ||
        instance.first_step_done.load(std::memory_order_acquire))
    {
        job.fail("clone() must be called before warmup() and the first step(). "
                 "Complete detach_ovstage(), then reattach the source stage, "
                 "to re-clone after warmup.");
        return;
    }

    // A target cloned earlier on this attach would get a duplicate object under its path.
    for (const std::string& target : request.targets)
    {
        if (instance.cloned_target_paths.count(target) != 0)
        {
            job.fail("clone: target path was already cloned on this attach");
            return;
        }
    }

    // Reserve target bookkeeping before cloning changes simulation state. Publishing the
    // successful target set is allocation-free.
    instance.cloned_target_paths.reserve(instance.cloned_target_paths.size() + request.targetSet.size());
    omni::physx::RuntimeErrorScope runtimeError;
    const bool cloned = physxSim->cloneEnvironments(
        instance.attachHandle, request.source.c_str(), request.targetPointers.data(),
        static_cast<uint32_t>(request.targetPointers.size()),
        request.anchorTransforms.empty() ? nullptr : request.anchorTransforms.data(),
        request.envIds.empty() ? nullptr : request.envIds.data(), request.useEnvIds);
    if (!cloned)
    {
        job.fail(runtimeError.message()[0] ?
                     runtimeError.message() :
                     "cloneEnvironments failed: invalid source, occupied target, or missing backing stage");
        return;
    }
    instance.cloned_target_paths.merge(request.targetSet);

    // A tensor backend created before clone() cached the pre-clone actor population and
    // buffer sizes, so invalidate it after a successful clone. Keyed by attach handle
    // rather than stage id, because a stageless attach would otherwise keep pre-clone
    // views live against a changed population.
    if (instance.attachHandle != omni::physics::tensors::kNoAttach)
    {
        if (omni::physics::tensors::TensorApi* tensorApi = omni::physx::runtime::tryGetTensorApiInterface())
        {
            if (tensorApi->resetStage)
                tensorApi->resetStage(instance.attachHandle);
        }
    }
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_clone(ovphysx_instance_handle_t handle,
                                                   ovphysx_string_t source_path_in_usd,
                                                   ovphysx_string_t* target_paths,
                                                   uint32_t num_target_paths,
                                                   const float* anchor_transforms,
                                                   const uint32_t* env_ids) {
    OVPHYSX_API_PROFILE_ZONE(clone);
    try {
        // Clone the source subtree to the target paths via the PhysX SDK replicator
        // (IPhysxSimulation::cloneEnvironments). Only path strings and a flat
        // [num_target_paths * 7] transform array cross the C ABI. anchor_transforms[i] is
        // the world pose of target_paths[i] (NULL co-locates on the source), and env_ids[i]
        // identifies copy i's logical environment (NULL means per-call numbering).

        // Argument failures reject the call without a receipt; checks that depend on
        // earlier work run in the job.
        if (!source_path_in_usd.ptr || source_path_in_usd.length == 0 || !target_paths || num_target_paths == 0) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     "clone: source_path_in_usd must be non-empty, target_paths non-null, "
                                     "and num_target_paths > 0");
        }

        // The clone is refused inside a callback, as every call that queues or waits for
        // instance work is.
        const ovphysx::Caller caller = ovphysx::currentCaller();
        if (caller.callback)
            return {ovphysx::refuseBlockingReentry(nullptr, caller), 0, 0};
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor) {
            return set_enqueue_error(OVPHYSX_API_NOT_FOUND, "Invalid instance handle");
        }
        if (ovphysx::refuseBlockingReentry(instance.get(), caller) != OVPHYSX_API_SUCCESS)
            return {OVPHYSX_API_INVALID_STATE, 0, 0};

        omni::physx::IPhysxSimulation* physxSim =
            instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
        if (!physxSim || !physxSim->cloneEnvironments) {
            return set_enqueue_error(OVPHYSX_API_ERROR, "clone: IPhysxSimulation::cloneEnvironments is unavailable");
        }

        // Copy every input: the job runs after the call returns. Views are not guaranteed
        // null-terminated and the runtime entry takes C strings, so the copies are
        // null-terminated strings with a stable array of target C-string pointers.
        std::shared_ptr<CloneRequest> request = std::make_shared<CloneRequest>();
        request->source.assign(source_path_in_usd.ptr, source_path_in_usd.length);
        if (request->source.find('\0') != std::string::npos) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     "clone: source_path_in_usd must not contain an embedded NUL");
        }
        request->targets.reserve(num_target_paths);
        request->targetPointers.reserve(num_target_paths);
        // Per-target validation (each target valid and unique in the call). Enforced C-first
        // so a bad target cannot create a duplicate PhysX object under an existing path. A
        // target already cloned on this attach is rejected by the job.
        request->targetSet.reserve(num_target_paths);
        for (uint32_t i = 0; i < num_target_paths; ++i) {
            if (!target_paths[i].ptr || target_paths[i].length == 0) {
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "clone: target path must be non-empty");
            }
            std::string target(target_paths[i].ptr, target_paths[i].length);
            // The length-tagged view can carry an embedded NUL. The seam takes a C string, so
            // c_str() would truncate and could silently alias the source. Reject it before the
            // length-based source comparison below.
            if (target.find('\0') != std::string::npos) {
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                         "clone: target path must not contain an embedded NUL");
            }
            if (target == request->source) {
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                         "clone: target path must differ from the source path");
            }
            if (!request->targetSet.insert(target).second) {
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                         "clone: duplicate target path in the same clone() call");
            }
            request->targets.push_back(std::move(target));
        }
        // The strings no longer move, so their C strings stay valid for the job.
        for (const std::string& target : request->targets)
            request->targetPointers.push_back(target.c_str());

        // Logical env id maps to runtime id env_ids[i] + 1 (0 is the source's). PhysX requires
        // every environment id < 1<<24 (setEnvironmentID), so the caller id must be < 0x00FFFFFF.
        // A larger value would silently make setEnvironmentID fail and the body collide with all.
        if (env_ids) {
            for (uint32_t i = 0; i < num_target_paths; ++i) {
                if (env_ids[i] >= 0x00FFFFFFu) {
                    return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                             "clone: env_ids values must be < 0x00FFFFFF (16777215); "
                                             "PhysX supports at most 1<<24 environments and the runtime "
                                             "id is env_ids[i] + 1");
                }
            }
            request->envIds.assign(env_ids, env_ids + num_target_paths);
        }
        if (anchor_transforms)
            request->anchorTransforms.assign(anchor_transforms,
                                             anchor_transforms + static_cast<size_t>(num_target_paths) * 7u);

        // env-id cross-environment collision filtering is a per-process setting (default on),
        // read when the call is made. With explicit transforms the copies are already
        // physically separated, so it is an optional add-on.
        if (carb::Framework* fw = carb::getFramework()) {
            if (carb::settings::ISettings* settings = fw->tryAcquireInterface<carb::settings::ISettings>()) {
                settings->setDefaultBool("/ovphysx/clone/useEnvIds", true);
                request->useEnvIds = settings->getAsBool("/ovphysx/clone/useEnvIds");
            }
        }

        // The clone runs as one job on the instance worker after every earlier job.
        const ovphysx::SubmittedAttachment submitted = ovphysx::submittedAttachment(*instance);
        std::unique_ptr<ovphysx::InstanceJob> job(new ovphysx::InstanceJob(
            "ovphysx_clone", handle, instance, [physxSim, request, submitted](ovphysx::InstanceJob& self) {
                run_clone_job(self, physxSim, *request, submitted);
            }));
        return ovphysx::submitInstanceJob(*instance, handle, std::move(job));
    } catch (const std::exception& error) {
        return set_enqueue_error(OVPHYSX_API_ERROR, error.what());
    } catch (...) {
        return set_enqueue_error(OVPHYSX_API_ERROR, "Unknown exception during clone");
    }
}

// The detach every path shares, under the lane(s) its caller holds: a detach job on the
// instance worker, the calling thread of a detach while a recording is active, destroy,
// and the rollback of a failed attach. The callers also hold companion admission and the
// lifecycle lock; this body takes none of them again.
static ovphysx_result_t detach_ovstage_body(ovphysx_instance_handle_t handle, InstanceData& instance) {
    const bool hadOvstage = instance.ovstage_attached;

    if (hadOvstage) {
        const int64_t stageId = instance.attachedStageId;
        const omni::physics::tensors::AttachHandle attachHandle = instance.attachHandle;

        // ovx_primpath_t handles are owned by the attached Stage's path
        // dictionary. Drop this attachment's visualization scope and this
        // instance's stored scope before the runtime detaches (and the caller
        // may destroy that dictionary). A replacement stage must start unscoped.
        instance.visualizationScopeDropped = true;
        clearVisualizationScopeTokens(attachHandle);
        omni::physx::ovxDebugRenderNoteAttach(handle, 0, 1);

        cleanup_output_writes(instance);

        // Release SDF views before PhysX/tensor teardown so native ISdfShapeView
        // objects do not outlive the per-stage SimulationBackend data.
        ovphysx_sdf_view_cleanup_instance(&instance);

        // Release the tensor SimulationBackend's data for this attach before
        // detachStage() destroys the AttachedStage that the views borrow. Keyed by
        // the attach handle and not gated on a nonzero stage id, because a
        // stageless attach needs this most of all.
        if (omni::physics::tensors::TensorApi* tensorApi =
                omni::physx::runtime::tryGetTensorApiInterface()) {
            if (tensorApi->resetStage)
                tensorApi->resetStage(attachHandle);
        }

        omni::physx::IPhysxSimulation* physxSim =
            instance.carbonite ? instance.carbonite->getPhysxSimulation() : nullptr;
        {
            std::lock_guard<std::mutex> attachLock(g_gpuAttachMutex);
            if (physxSim && physxSim->detachStage) {
                if (checkRuntimeResult(physxSim->detachStage(attachHandle), "detachStage") != OVPHYSX_API_SUCCESS)
                    return set_error(OVPHYSX_API_ERROR, "detach_ovstage: runtime rejected detach");
                ovphysx_object_change_detached_instance(handle);
                finalize_recording_after_detach(physxSim, handle);
            }
        }

        if (stageId != 0) {
            markStageDetached(stageId, "detach_ovstage");
        }
        {
            std::unique_lock<std::shared_mutex> liveLock(g_instances_mutex);
            instance.attachedStageId = 0;
            instance.attachHandle = omni::physics::tensors::kNoAttach;
            instance.ovstage_attached = false;
        }
        instance.ovstage_attach_payload = OvstageAttachPayload{};
        instance.resetStageFlags();
    }

    return success();
}

// The body of a detach job, run by the lane holder. generation is
// the attachment generation the detach took at submission.
static void run_detach_job(ovphysx::InstanceJob& job, uint64_t generation)
{
    std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
    if (detach_ovstage_body(job.handle(), job.instance()).status != OVPHYSX_API_SUCCESS)
    {
        const ThreadLocalError& error = tls_error();
        job.fail(error.last_error.empty() ? "detach_ovstage: runtime rejected detach" : error.last_error.c_str());
        return;
    }
    job.instance().executed_generation.store(generation, std::memory_order_release);
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_detach_ovstage(ovphysx_instance_handle_t handle) {
    OVPHYSX_API_PROFILE_ZONE(detach_ovstage);
    TaskContext admission;
    try {
        // Refused before the companion checks, so a callback changes nothing.
        const ovphysx::Caller caller = ovphysx::currentCaller();
        if (caller.callback)
            return {ovphysx::refuseBlockingReentry(nullptr, caller), 0, 0};
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "detach_ovstage: invalid handle");
        }
        if (ovphysx::refuseBlockingReentry(instance.get(), caller) != OVPHYSX_API_SUCCESS)
            return {OVPHYSX_API_INVALID_STATE, 0, 0};
        // Reserve companion transition admission at submission. Operations already
        // running reject this call; later operations stay busy until the lifecycle
        // work finishes.
        std::shared_ptr<CompanionJobTransitionGuard> companionJobTransition =
            std::make_shared<CompanionJobTransitionGuard>(companionRuntime());
        if (!*companionJobTransition)
            return set_enqueue_error(OVPHYSX_API_INVALID_STATE, kCompanionTransitionBusy);

        // The detach runs as one job on the instance worker after every earlier job. It takes
        // the next attachment generation once it is accepted (see InstanceData).
        const uint64_t generation = instance->lifecycle_generations + 1;
        std::unique_ptr<ovphysx::InstanceJob> job(new ovphysx::InstanceJob(
            "ovphysx_detach_ovstage", handle, instance,
            [generation, companionJobTransition](ovphysx::InstanceJob& self) mutable {
                struct ReleaseTransition
                {
                    std::shared_ptr<CompanionJobTransitionGuard>& value;
                    ~ReleaseTransition() { value.reset(); }
                } release{ companionJobTransition };
                run_detach_job(self, generation);
            }));
        companionJobTransition.reset(); // The job now owns the reservation.
        {
            std::unique_lock<std::mutex> recordingLock(g_recordingMutex);
            if (!lifecycle_runs_on_caller())
            {
                const ovphysx_enqueue_result_t receipt = ovphysx::queueInstanceJob(*instance, handle, std::move(job));
                if (receipt.status == OVPHYSX_API_SUCCESS)
                    ovphysx::noteLifecycleSubmission(*instance, generation, receipt.op_index);
                recordingLock.unlock();
                return ovphysx::runWithoutWorker(*instance, receipt);
            }
        }

        // While a recording is active the detach runs here, under every lane, then companion
        // transition admission, then the lifecycle lock, and returns a finished task. Every
        // instance's lane finishes peer work without collecting its task results.
        ovphysx::AllInstanceLanes everyLane;
        ovphysx_api_status_t wait_status = everyLane.acquire();
        if (wait_status == OVPHYSX_API_INVALID_STATE)
            return {wait_status, 0, 0};
        if (wait_status == OVPHYSX_API_SUCCESS && get_instance(handle) != instance)
            wait_status = OVPHYSX_API_NOT_FOUND;
        if (wait_status != OVPHYSX_API_SUCCESS)
            return set_enqueue_error(wait_status, "Failed to complete pending operations before detach_ovstage");
        CompanionTransitionGuard companionTransition(companionRuntime());
        if (!companionTransition)
            return set_enqueue_error(OVPHYSX_API_INVALID_STATE, kCompanionTransitionBusy);
        if (companionRuntime().hasPendingPreviews())
        {
            std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
            if (g_omniPvdRecordingOwner != OVPHYSX_INVALID_HANDLE)
                return set_enqueue_error(OVPHYSX_API_INVALID_STATE,
                                         "Fetch companion preview results before recording detach");
        }
        // With every lane held no job of this instance runs, so its attachment is final.
        // Owner and last-attachment detach close the shared stream, which also needs the
        // runtime idle. Release the transition lock before taking g_lifecycleMutex:
        // detach_ovstage_body reacquires g_gpuAttachMutex in lifecycle-before-GPU order.
        bool finalizesRecording = false;
        {
            std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
            finalizesRecording = instance->ovstage_attached && g_omniPvdRecordingOwner != OVPHYSX_INVALID_HANDLE &&
                                 (g_omniPvdRecordingOwner == handle || !other_instance_attached(handle));
        }
        if (finalizesRecording)
        {
            std::unique_lock<std::mutex> transitionLock(g_gpuAttachMutex, std::defer_lock);
            if (enter_shared_runtime_safe_point(transitionLock) != OVPHYSX_API_SUCCESS)
                return set_enqueue_error(OVPHYSX_API_INVALID_STATE,
                                         "Failed to complete pending operations before detach_ovstage");
        }
        ovphysx_result_t result{ OVPHYSX_API_ERROR };
        {
            std::lock_guard<std::mutex> lifecycle(g_lifecycleMutex);
            admission.create_task(handle);
            ovphysx::noteLifecycleSubmission(*instance, generation, admission.receipt.op_index);
            result = detach_ovstage_body(handle, *instance);
            if (result.status == OVPHYSX_API_SUCCESS)
                instance->executed_generation.store(generation, std::memory_order_release);
        }
        // As on the queued path, companion admission ends before task publication.
        job.reset();
        if (result.status != OVPHYSX_API_SUCCESS)
            return admission.fail(ovphysx_get_last_error().ptr);
        return admission.complete();
    } catch (const std::exception& error) {
        return admission.fail(error.what());
    } catch (...) {
        return admission.fail("Unknown exception during detach_ovstage");
    }
}

OVPHYSX_API ovphysx_result_t ovphysx_is_stage_attached(ovphysx_instance_handle_t instance_handle,
                                                    const void* stage, bool* out_attached) {
    OVPHYSX_API_PROFILE_ZONE(is_stage_attached);
    if (!out_attached)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_stage_attached: out_attached is NULL");
    *out_attached = false;

    // Attach/detach alone change this state; no simulation wait is needed.
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(instance_handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_stage_attached: invalid handle");

    *out_attached = instance->ovstage_attached && (!stage || instance->ovstage_attach_payload.instance == stage);
    return success();
}

// Private Python binding ABI: session lifetime checks must not wait or collect tasks.
extern "C" OVPHYSX_API ovphysx_result_t ovphysx_internal_get_attach_handle(
    ovphysx_instance_handle_t instance_handle, uint64_t* out_attach_handle)
{
    if (!out_attach_handle)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "internal_get_attach_handle: output is NULL");
    *out_attach_handle = omni::physics::tensors::kNoAttach;
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(instance_handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "internal_get_attach_handle: invalid handle");
    *out_attach_handle = static_cast<uint64_t>(instance->attachHandle);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_find_prims(ovphysx_instance_handle_t handle,
                                                        ovphysx_string_t path_pattern,
                                                        ovphysx_string_t attribute_name,
                                                        ovphysx_prim_list_t* out_prim_list) {
    ovphysx::InstanceLane lane;
    ovphysx_api_status_t wait_status = lane.acquire(handle);
    if (wait_status == OVPHYSX_API_INVALID_STATE) {
        return {wait_status};
    }
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before find_prims");
    }

    return set_error(OVPHYSX_API_NOT_IMPLEMENTED, "find_prims not yet implemented");
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_prim_list(ovphysx_instance_handle_t handle,
                                                               ovphysx_prim_list_t* prim_list) {
    ovphysx::InstanceLane lane;
    ovphysx_api_status_t wait_status = lane.acquire(handle);
    if (wait_status == OVPHYSX_API_INVALID_STATE) {
        return {wait_status};
    }
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before destroy_prim_list");
    }

    // No-op for now
    if (prim_list) {
        prim_list->prim_paths = nullptr;
        prim_list->num_paths = 0;
    }
    return success();
}

// @implements REQ-CAPI-TASK-001
// @covers AC-2 AC-3 AC-4 AC-5 AC-7 AC-8 AC-9
static bool wait_output_is_empty(const ovphysx_wait_task_result_t& result)
{
    return !result.error_tasks && result.num_errors == 0 &&
        result.pending_task.status == OVPHYSX_API_SUCCESS &&
        result.pending_task.op_index == 0 && result.pending_task.instance == 0;
}

// The task table lives in the instance executor. Worker jobs publish outcomes
// concurrently, so a wait holds the executor mutex while a job may be queued or
// running. With the lane idle, nothing unfinished remains and the worker cannot
// publish before this externally serialized caller submits again, so the wait reads
// and collects without the mutex and without a deadline. A wait without a deadline on a
// native thread first runs, on that thread, the jobs it waits for that the worker has not
// started (InstanceExecutor::runUnstartedJobs()), unless a CUDA graph capture holds the
// legacy default stream of its current context, where any CUDA work could invalidate the
// capture. Otherwise a wait observes job completion through the executor's condition
// variable (task-aware inside a Carbonite task), so a finite wait or a poll never runs
// work, ends at its deadline and reports an outcome published by then.
static ovphysx_result_t wait_tasks(
    ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task, bool wait_all,
    ovphysx_timeout_t timeout_ns, ovphysx_wait_task_result_t* output)
{
    try {
        // Instance validation precedes receipt and output validation.
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor)
            return set_error(OVPHYSX_API_NOT_FOUND, "Invalid instance handle");
        // A callback may run inside the work this wait would wait for.
        if (ovphysx::refuseBlockingReentry(instance.get()) != OVPHYSX_API_SUCCESS)
            return {OVPHYSX_API_INVALID_STATE};
        if (companionRuntime().nativeAccessOnThisThread())
            return set_error(OVPHYSX_API_INVALID_STATE, "Cannot wait or collect tasks inside a companion native-access lease");
        if (!output || !wait_output_is_empty(*output))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Wait result must be non-null and zero-initialized");
        ovphysx::InstanceExecutor& executor = *instance->executor;
        ovphysx::InstanceExecutor::TaskTable& table = executor.tasks();
        const bool idle = executor.laneIdle();
        const bool runsJobs = !idle && timeout_ns == OVPHYSX_TIMEOUT_INFINITE && !legacyStreamHeldByCapture();
        std::unique_lock<std::mutex> tasksLock;
        if (!idle)
            tasksLock = executor.lock();
        const ovphysx_op_index_t cutoff = table.nextIndex.load(std::memory_order_relaxed) - 1;
        if (!wait_all) {
            if (task.status != OVPHYSX_API_SUCCESS || !task.op_index || task.instance != handle)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Task is rejected, zero, or belongs to another instance");
            if (task.op_index > cutoff || table.isCollected(task.op_index))
                return set_error(OVPHYSX_API_NOT_FOUND, "Task was never issued or was already collected");
        }

        ovphysx_op_index_t pending = 0;
        if (!idle) {
            if (runsJobs)
                executor.runUnstartedJobs(tasksLock, wait_all ? cutoff : task.op_index);
            // Wait for publication within one deadline; only a finite timeout needs the
            // clock. TIMEOUT reports the earliest unfinished task within the cutoff.
            const bool finite = timeout_ns != OVPHYSX_TIMEOUT_POLL && timeout_ns != OVPHYSX_TIMEOUT_INFINITE;
            const ovphysx::WaitDeadline deadline = ovphysx::WaitDeadline::fromTimeoutNs(
                timeout_ns, finite ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{});
            if (wait_all) {
                executor.waitLocked(tasksLock, [&]() { return executor.firstUnfinished(cutoff) == 0; }, deadline);
                pending = executor.firstUnfinished(cutoff);
            } else {
                const ovphysx_op_index_t target = task.op_index;
                if (!executor.waitLocked(tasksLock, [&]() { return !executor.isUnfinished(target); }, deadline))
                    pending = target;
            }
        }

        const ovphysx_op_index_t first = wait_all ? table.collectedPrefix + 1 : task.op_index;
        const ovphysx_op_index_t last = wait_all ? (pending ? pending - 1 : cutoff)
                                               : (pending ? task.op_index - 1 : task.op_index);
        std::shared_ptr<TaskWaitStorage> storage;
        if (first <= last) {
            for (auto it = table.failures.lower_bound(first);
                 it != table.failures.end() && it->first <= last; ++it) {
                if (!storage)
                    storage = std::make_shared<TaskWaitStorage>();
                storage->tasks.push_back({OVPHYSX_API_SUCCESS, it->first, handle});
            }
            if (storage)
                storage->errors.resize(storage->error_count());
        }

        // Prepare every allocation before acknowledging anything. Retiring an
        // execution record above never acknowledges its terminal outcome.
        std::unique_ptr<std::shared_ptr<TaskWaitStorage>> owner;
        std::unique_lock<std::mutex> retainedLock;
        if (storage) {
            owner = std::make_unique<std::shared_ptr<TaskWaitStorage>>(storage);
            retainedLock = std::unique_lock<std::mutex>(instance->retained_wait_results_mutex);
            std::vector<std::weak_ptr<TaskWaitStorage>>& retained = instance->retained_wait_results;
            retained.erase(std::remove_if(retained.begin(), retained.end(),
                [](const std::weak_ptr<TaskWaitStorage>& result) { return result.expired(); }), retained.end());
            retained.reserve(retained.size() + 1);
        }

        table.collectRange(first, last);
        // The live result owns the collected messages; the instance registry is weak.
        // Everything after acknowledgement is allocation-free.
        if (storage) {
            for (size_t index = 0; index < storage->error_count(); ++index) {
                const auto error = table.failures.find(storage->error_tasks()[index].op_index);
                storage->errors[index].swap(error->second);
                table.failures.erase(error);
            }
            storage->set_owner(owner.release());
            instance->retained_wait_results.emplace_back(storage);
            output->error_tasks = storage->error_tasks();
            output->num_errors = storage->error_count();
        }
        if (retainedLock.owns_lock())
            retainedLock.unlock();
        if (tasksLock.owns_lock())
            tasksLock.unlock();
        if (pending)
            output->pending_task = {OVPHYSX_API_SUCCESS, pending, handle};
        if (pending) {
            success();
            return {OVPHYSX_API_TIMEOUT};
        }
        if (storage)
            return set_error(OVPHYSX_API_ERROR, "One or more tasks failed");
        return success();
    } catch (const std::exception& error) {
        return set_error(OVPHYSX_API_ERROR, error.what());
    } catch (...) {
        return set_error(OVPHYSX_API_ERROR, "Internal task wait failure");
    }
}

OVPHYSX_API ovphysx_result_t ovphysx_wait_task(
    ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task,
    ovphysx_timeout_t timeout_ns, ovphysx_wait_task_result_t* output)
{
    OVPHYSX_API_PROFILE_ZONE(wait_task);
    return wait_tasks(handle, task, false, timeout_ns, output);
}

OVPHYSX_API ovphysx_result_t ovphysx_wait_all(
    ovphysx_instance_handle_t handle, ovphysx_timeout_t timeout_ns, ovphysx_wait_task_result_t* output)
{
    OVPHYSX_API_PROFILE_ZONE(wait_all);
    return wait_tasks(handle, {}, true, timeout_ns, output);
}

// @implements REQ-CAPI-TASK-001
// @covers AC-10
OVPHYSX_API ovphysx_result_t ovphysx_query_task_status(
    ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task, ovphysx_task_status_t* out_status)
{
    OVPHYSX_API_PROFILE_ZONE(query_task_status);
    if (!out_status)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "query_task_status: out_status is NULL");
    *out_status = OVPHYSX_TASK_STATUS_NOT_FOUND;
    try {
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance || !instance->executor)
            return set_error(OVPHYSX_API_NOT_FOUND, "Invalid instance handle");
        if (task.status != OVPHYSX_API_SUCCESS || !task.op_index || task.instance != handle)
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Task is rejected, zero, or belongs to another instance");
        std::unique_lock<std::mutex> tasksLock = instance->executor->lock();
        const ovphysx::InstanceExecutor::TaskTable& table = instance->executor->tasks();
        if (task.op_index >= table.nextIndex || table.isCollected(task.op_index))
            return success();
        switch (instance->executor->taskState(task.op_index)) {
        case ovphysx::InstanceExecutor::TaskState::kPending:
            *out_status = OVPHYSX_TASK_STATUS_PENDING;
            break;
        case ovphysx::InstanceExecutor::TaskState::kRunning:
            *out_status = OVPHYSX_TASK_STATUS_RUNNING;
            break;
        case ovphysx::InstanceExecutor::TaskState::kSucceeded:
            *out_status = OVPHYSX_TASK_STATUS_SUCCEEDED;
            break;
        case ovphysx::InstanceExecutor::TaskState::kFailed:
            *out_status = OVPHYSX_TASK_STATUS_FAILED;
            break;
        case ovphysx::InstanceExecutor::TaskState::kNotFound:
            break;
        }
        return success();
    } catch (const std::exception& error) {
        return set_error(OVPHYSX_API_ERROR, error.what());
    } catch (...) {
        return set_error(OVPHYSX_API_ERROR, "Internal task status failure");
    }
}

OVPHYSX_API ovphysx_string_t ovphysx_get_last_error(void)
{
    OVPHYSX_API_PROFILE_ZONE(get_last_error);
    const ThreadLocalError& error = tls_error();
    if (error.fallback_error.ptr)
        return error.fallback_error;
    return {error.last_error.c_str(), error.last_error.size()};
}

OVPHYSX_API ovphysx_string_t ovphysx_get_last_task_error(
    ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task)
{
    OVPHYSX_API_PROFILE_ZONE(get_last_task_error);
    try {
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance) {
            set_error(OVPHYSX_API_NOT_FOUND, "Invalid instance handle");
            return {"", 0};
        }
        if (task.status != OVPHYSX_API_SUCCESS || !task.op_index || task.instance != handle) {
            set_error(OVPHYSX_API_INVALID_ARGUMENT, "Task is rejected, zero, or belongs to another instance");
            return {"", 0};
        }
        std::lock_guard<std::mutex> retainedLock(instance->retained_wait_results_mutex);
        for (const std::weak_ptr<TaskWaitStorage>& retained : instance->retained_wait_results) {
            const std::shared_ptr<TaskWaitStorage> storage = retained.lock();
            if (!storage)
                continue;
            const ovphysx_enqueue_result_t* const first = storage->error_tasks();
            const ovphysx_enqueue_result_t* const last = first + storage->error_count();
            const ovphysx_enqueue_result_t* const found = std::lower_bound(first, last, task.op_index,
                [](const ovphysx_enqueue_result_t& entry, ovphysx_op_index_t index) { return entry.op_index < index; });
            if (found != last && found->op_index == task.op_index) {
                const std::string& message = storage->errors[static_cast<size_t>(found - first)];
                static constexpr char fallback[] = "Task execution failed";
                return message.empty() ? ovphysx_string_t{fallback, sizeof(fallback) - 1}
                                       : ovphysx_string_t{message.c_str(), message.size()};
            }
        }
    } catch (const std::exception& error) {
        set_error(OVPHYSX_API_ERROR, error.what());
    } catch (...) {
        set_error(OVPHYSX_API_ERROR, "Internal task error lookup failure");
    }
    return {"", 0};
}

OVPHYSX_API void ovphysx_destroy_wait_task_result(ovphysx_wait_task_result_t* result)
{
    OVPHYSX_API_PROFILE_ZONE(destroy_wait_task_result);
    if (result) {
        delete TaskWaitStorage::owner_for(result->error_tasks);
        *result = {};
    }
}

} // extern "C"

// Log capture API and logging configuration are implemented in LogManager.cpp
