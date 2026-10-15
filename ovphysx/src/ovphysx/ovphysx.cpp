// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5

/**
 * @implements REQ-CAPI-HOST-EXIT-001
 * @covers AC-1 AC-2
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
 * @implements REQ-CAPI-TIMESTEP-002
 * @covers AC-1
 *
 * @implements REQ-CAPI-THREADS-001
 * @covers AC-1 AC-2
 */

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-5 AC-6
 */

/**
 * @implements REQ-CAPI-TASK-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8
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
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9 AC-10 AC-11
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
 * @covers AC-1 AC-2 AC-3 AC-4
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
 * @covers AC-1 AC-2 AC-3
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
 * @covers AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-CONFIG-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-9
 */

#include "internal/CpuFeatureCheck.h"
#include "LogManager.hpp"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "cuda_shim/CudaShim.h"
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
#include <charconv>
#include <cmath>
#include <cstring>
#include <vector>
#include <atomic>
#include <shared_mutex>
#include <cinttypes> // For PRIu64 portable format specifier

// Platform-specific includes (Windows/Linux dynamic loading)
#include "internal/sdk/PlatformIncludes.hpp"
#include "internal/sdk/ovphysxAsyncWait.hpp"
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
// that still hold it after erase.
std::unordered_map<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>> g_instances;

// Guards g_instances. Lookups take shared_lock, insert/erase take unique_lock.
std::shared_mutex g_instances_mutex;

namespace
{

// The one process-wide serial sequence behind every ovphysx-owned opaque object
// handle: instances, tensor bindings, contact bindings and SDF views.
//
// It is process-wide on purpose (NVBug 6504951). Independent per-kind counters
// share the same numeric space, so an instance handle and that instance's first
// tensor binding would both be 1, and a binding handle from a destroyed instance
// would match the first binding of the next one. One never-reused sequence keeps
// the four handle kinds numerically unique for the life of the process. Handles
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

// Serializes the process-global attach-time GPU selector with scene attachment
// so simultaneous attach calls from different handles cannot overwrite it while
// PhysX consumes it.
std::mutex g_gpuAttachMutex;

// The public owner of the shared runtime's one active OmniPVD stream. This is
// separate from stage ownership: a peer handle may start recording the one
// attached stage. Recording transitions share g_gpuAttachMutex with attach and
// detach so a stream cannot be rebound while its stage is being torn down.
static ovphysx_instance_handle_t g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;

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
// warmup(), and create_tensor_binding().
ovphysx_api_status_t ovphysx_ensure_physics_attached(ovphysx_instance_handle_t handle)
{
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return OVPHYSX_API_ERROR;

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

// Starts a simulation step without waiting for results. Non-static because
// ovphysxTensorBinding.cpp uses it for auto-warmup.
ovphysx_api_status_t omni_sdk_physx_simulate_instance(ovphysx_instance_handle_t handle, float elapsedTime, float currentTime,
                                                        ovphysx_op_index_t task_index) {
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
        {
            ovphysx_api_status_t attach_status = ovphysx_ensure_physics_attached(handle);
            if (attach_status != OVPHYSX_API_SUCCESS)
                return attach_status;
        }
        
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (!instance) {
            CARB_LOG_ERROR("[PHYSICS SIMULATION] ERROR: Invalid instance handle: %" PRIu64, handle);
            return OVPHYSX_API_ERROR;
        }

        // Get the PhysX interface before dispatching work.
        auto physxSim = instance->carbonite->getPhysxSimulation();
        if (!physxSim) {
            CARB_LOG_ERROR("[PHYSICS SIMULATION] ERROR: PhysX simulation interface not available");
            return OVPHYSX_API_ERROR;
        }

        // Keep map lock held while locking the instance mutex to prevent iterator
        // invalidation during map rehash.
        std::unique_lock<std::mutex> instance_lock(instance->simulationMutex);

        // Callers finish the preceding simulation before dispatching another.
        auto& pending = instance->pending_simulation;
        if (pending.active)
            return set_error(OVPHYSX_API_INVALID_STATE, "A simulation is already pending").status;
        pending = {};
        pending.task_index = task_index;
        try {
            omni::physx::RuntimeErrorScope runtimeError;
            if (checkRuntimeResult(physxSim->simulate(instance->attachHandle, elapsedTime, currentTime), "simulate") != OVPHYSX_API_SUCCESS)
                return OVPHYSX_API_ERROR;
            pending.active = true;
            if (instance->attachHandle != omni::physics::tensors::kNoAttach)
                instance->first_step_done.store(true, std::memory_order_release);
            if (runtimeError.message()[0]) {
                pending.fail(runtimeError.message());
                return set_error(OVPHYSX_API_ERROR, runtimeError.message()).status;
            }
            return OVPHYSX_API_SUCCESS;
        } catch (const std::exception& error) {
            return set_error(OVPHYSX_API_ERROR, error.what()).status;
        } catch (...) {
            return set_error(OVPHYSX_API_ERROR, "Unknown exception during simulate").status;
        }
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

    // @implements REQ-CAPI-CONFIG-001
    // @covers AC-4 AC-7
    static ovphysx_api_status_t updateLiveContactProcessing(bool disabled)
    {
        const std::vector<LiveAttachment> live = collect_live_attachments();
        if (live.empty())
            return OVPHYSX_API_SUCCESS;
        if (omni::physx::runtime::isRuntimeExecuting())
            return OVPHYSX_API_INVALID_STATE;
        for (const LiveAttachment& attachment : live)
        {
            const ovphysx_api_status_t drain = omni_sdk_physx_wait_all_pending_internal(attachment.handle);
            if (drain != OVPHYSX_API_SUCCESS)
                return drain;
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

        return configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdOutputEnabled) ||
               configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdRecordingCapable) ||
               configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdOvdRecordingDirectory) ||
               configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTransport) ||
               configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTcpAddress) ||
               configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTcpPort) ||
               configCarbonitePathEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTcpTimeoutMs);
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
    // Clamp ovphysx timeout values to std::chrono::nanoseconds range.
    static std::chrono::nanoseconds clamp_timeout_ns(ovphysx_timeout_t timeout_ns) {
        using ns = std::chrono::nanoseconds;
        const uint64_t ns_max = static_cast<uint64_t>(ns::max().count());
        if (timeout_ns >= ns_max) {
            return ns::max();
        }
        return ns(static_cast<ns::rep>(timeout_ns));
    }

    // Readiness finishes execution; public waits separately collect its outcome.
    static ovphysx_api_status_t finish_pending_simulation(
        InstanceData& instance, ovphysx_timeout_t timeout_ns,
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now())
    {
        std::unique_lock<std::mutex> lock(instance.simulationMutex);
        auto& pending = instance.pending_simulation;
        if (!pending.active)
            return OVPHYSX_API_SUCCESS;
        auto physxSim = instance.carbonite ? instance.carbonite->getPhysxSimulation() : nullptr;
        if (!physxSim) {
            pending.fail("PhysX simulation interface not available");
        } else if (instance.attachHandle != omni::physics::tensors::kNoAttach) {
            try {
                if (timeout_ns != OVPHYSX_TIMEOUT_INFINITE &&
                    !ovphysx::async::detail::wait_until_simulation_ready(
                        timeout_ns == OVPHYSX_TIMEOUT_POLL, clamp_timeout_ns(timeout_ns), start,
                        [physxSim, &instance]() { return physxSim->checkResults(instance.attachHandle); },
                        []() { return std::chrono::steady_clock::now(); },
                        [](std::chrono::steady_clock::duration duration) { std::this_thread::sleep_for(duration); }))
                    return OVPHYSX_API_TIMEOUT;
                omni::physx::RuntimeErrorScope runtimeError;
                if (physxSim->fetchResults(instance.attachHandle) != omni::physx::RuntimeResult::eSuccess)
                    pending.fail("fetchResults rejected by the runtime");
                if (runtimeError.message()[0])
                    pending.fail(runtimeError.message());
            } catch (const std::exception& error) {
                pending.fail(error.what());
            } catch (...) {
                pending.fail("Unknown exception during fetchResults");
            }
        }
        const bool failed = pending.failed;
        if (failed)
            set_error(OVPHYSX_API_ERROR, pending.error.empty() ? "Task execution failed" : pending.error.c_str());
        if (pending.task_index) {
            if (failed) {
                auto error = instance.task_errors.find(pending.task_index);
                if (error != instance.task_errors.end())
                    error->second.swap(pending.error);
            } else {
                instance.task_errors.erase(pending.task_index);
            }
        }
        pending = {};
        return failed ? OVPHYSX_API_ERROR : OVPHYSX_API_SUCCESS;
    }

    // Finishing earlier execution never collects its error or rejects later work
    // merely because that earlier task failed.
    static ovphysx_api_status_t wait_for_all_pending_ops(ovphysx_instance_handle_t handle)
    {
        const auto instance = get_instance(handle);
        if (!instance)
            return OVPHYSX_API_NOT_FOUND;
        try {
            if (instance->pending_simulation.active)
                finish_pending_simulation(*instance, OVPHYSX_TIMEOUT_INFINITE);
            return OVPHYSX_API_SUCCESS;
        } catch (const std::exception& error) {
            set_error(OVPHYSX_API_ERROR, error.what());
        } catch (...) {
            set_error(OVPHYSX_API_ERROR, "Internal execution drain failed");
        }
        return OVPHYSX_API_ERROR;
    }

    // Recording mutates the one process-wide PhysX/OmniPVD runtime. Drain every
    // live attachment's pending work, plus the caller when it is not attached,
    // before the transition lock is held. The runtime reports Busy if an owner
    // is still in flight; this drain covers ovphysx task receipts.
    static ovphysx_api_status_t acquire_shared_runtime_safe_point(
        ovphysx_instance_handle_t caller, std::unique_lock<std::mutex>& transitionLock)
    {
        if (companionRuntime().hasPendingPreviews() || omni::physx::runtime::isRuntimeExecuting())
            return OVPHYSX_API_INVALID_STATE;

        const std::vector<LiveAttachment> live = collect_live_attachments();
        ovphysx_api_status_t status = OVPHYSX_API_SUCCESS;
        bool sawCaller = false;
        for (const LiveAttachment& attachment : live)
        {
            if (attachment.handle == caller)
                sawCaller = true;
            const ovphysx_api_status_t one = wait_for_all_pending_ops(attachment.handle);
            if (status == OVPHYSX_API_SUCCESS)
                status = one;
        }
        if (!sawCaller)
        {
            const ovphysx_api_status_t callerStatus = wait_for_all_pending_ops(caller);
            if (status == OVPHYSX_API_SUCCESS)
                status = callerStatus;
        }

        transitionLock.lock();
        return status;
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
            g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;
    }

    // Recording is process-wide. The instance that started it can end it, and so
    // can detach of the last live attachment: that teardown is the recorded
    // scene's destruction, even when a stageless peer started the stream.
    // Detaching one attachment while another stays live does not stop a stream
    // that this instance did not start.
    static void finalize_recording_after_detach(
        omni::physx::IPhysxSimulation* physxSim, ovphysx_instance_handle_t handle)
    {
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
            out.boolValue = true;
            return true;
        default:
            return false;
        }
    }
}

ovphysx_api_status_t omni_sdk_physx_wait_all_pending_internal(ovphysx_instance_handle_t handle)
{
    return wait_for_all_pending_ops(handle);
}

ovphysx_api_status_t omni_sdk_physx_finish_warmup(ovphysx_instance_handle_t handle)
{
    const auto instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_NOT_FOUND, "Invalid warmup instance").status;
    return finish_pending_simulation(*instance, OVPHYSX_TIMEOUT_INFINITE);
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

static ovphysx_result_t detach_ovstage_internal(ovphysx_instance_handle_t handle, TaskContext* admission);

ovphysx_api_status_t omni_sdk_physx_destroy(
    ovphysx_instance_handle_t handle, std::shared_ptr<ovphysx::CarboniteLoader>& deferredCarbonite)
{
    // Reject an already-absent handle before any process-global teardown. This
    // defensive path lets Python recover from an ambiguous FFI exception
    // without disturbing live peers or their asynchronous state.
    {
        std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
        std::unordered_map<ovphysx_instance_handle_t, std::shared_ptr<InstanceData>>::const_iterator it =
            g_instances.find(handle);
        if (it == g_instances.end() || !it->second)
            return OVPHYSX_API_ERROR;
    }

    // Destruction can finalize a stream owned by a peer of the attached stage, so
    // establish the same shared-runtime safe point as explicit start/stop before
    // any teardown. A busy safe point returns INVALID_STATE without consuming the
    // owner; terminal teardown failures still make best-effort progress.
    bool recordingOwnedAtDestroy = false;
    {
        std::unique_lock<std::mutex> transitionLock(g_gpuAttachMutex, std::defer_lock);
        {
            std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
            recordingOwnedAtDestroy = g_omniPvdRecordingOwner == handle;
        }
        if (recordingOwnedAtDestroy)
        {
            const ovphysx_api_status_t status = acquire_shared_runtime_safe_point(handle, transitionLock);
            if (status == OVPHYSX_API_INVALID_STATE) return status;
        }
    }

    if (const auto instance = get_instance(handle))
        cleanup_output_sessions(handle, *instance);

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
        const ovphysx_result_t detachResult = detach_ovstage_internal(handle, nullptr);
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
            // (for example wait_for_all_pending_ops() failed), so this handle may
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
                g_omniPvdRecordingOwner = handle;
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

    std::lock_guard<std::mutex> createLock(g_createInstanceMutex);

    bool isFirstLiveInstance = false;
    {
        std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
        isFirstLiveInstance = g_instances.empty() && !companionRuntime().hasServiceLeases();
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
            // Standalone recording starts only through the explicit API. Clear
            // inherited startup output without changing an existing session.
            if (isFirstLiveInstance)
            {
                if (settings->getAsBool(omni::physx::kOmniPvdOutputEnabled))
                    CARB_LOG_WARN("[ovphysx] Ignoring inherited /physics/omniPvdOutputEnabled=true. %s",
                                  kRetiredOvdConfigError);
                settings->setBool(omni::physx::kOmniPvdOutputEnabled, false);
            }
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
            }
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

    {
        std::unique_lock<std::shared_mutex> map_lock(g_instances_mutex);
        g_instances[handle] = std::move(instanceData);
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

OVPHYSX_API ovphysx_result_t ovphysx_ovd_start_recording(
    ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination)
{
    OVPHYSX_API_PROFILE_ZONE(ovd_start_recording);
    CompanionTransitionGuard companionTransition(companionRuntime());
    if (!companionTransition)
        return set_error(OVPHYSX_API_INVALID_STATE, "Companion runtime operation is executing; retry transition");
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "start_recording: invalid handle");
    if (!destination)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "start_recording: destination is null");

    std::string filePath;
    std::string tcpAddress;
    if (!omni::physx::copyOmniPvdConfigString(
            destination->file_path.ptr, destination->file_path.length, filePath) ||
        !omni::physx::copyOmniPvdConfigString(
            destination->tcp_address.ptr, destination->tcp_address.length, tcpAddress))
    {
        return set_error(
            OVPHYSX_API_INVALID_ARGUMENT, "start_recording: destination strings must not contain embedded NUL bytes");
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
                "start_recording: FILE requires a non-empty file_path and empty/zero TCP fields");
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
                "start_recording: TCP requires an empty file_path, non-empty address, port in 1..65535, and non-negative timeout");
        }
        transport = 1;
        target = tcpAddress.c_str();
        tcpPort = static_cast<uint16_t>(destination->tcp_port);
        tcpTimeoutMs = static_cast<uint32_t>(destination->tcp_timeout_ms);
    }
    else
    {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "start_recording: invalid destination transport");
    }

    std::unique_lock<std::mutex> recordingLock(g_gpuAttachMutex, std::defer_lock);
    const ovphysx_api_status_t waitStatus = acquire_shared_runtime_safe_point(handle, recordingLock);
    if (waitStatus != OVPHYSX_API_SUCCESS)
        return set_error(waitStatus, "start_recording: failed to reach a shared-runtime safe point");

    omni::physx::IPhysxSimulation* physxSim =
        instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->startOmniPvdRecording)
        return set_error(OVPHYSX_API_ERROR, "start_recording: OmniPVD runtime API is unavailable");

    bool recording = false;
    if (!physxSim->getOmniPvdRecordingStatus ||
        physxSim->getOmniPvdRecordingStatus(&recording) != omni::physx::RuntimeResult::eSuccess)
        return set_error(OVPHYSX_API_ERROR, "start_recording: runtime status query failed");
    if (recording)
        return set_error(OVPHYSX_API_INVALID_STATE, "start_recording: a recording is already active");

    const omni::physx::OmniPvdRecordingResult result =
        physxSim->startOmniPvdRecording(transport, target, tcpPort, tcpTimeoutMs);
    switch (result)
    {
    case omni::physx::OmniPvdRecordingResult::eSuccess:
        g_omniPvdRecordingOwner = handle;
        return success();
    case omni::physx::OmniPvdRecordingResult::eInvalidState:
        return set_error(
            OVPHYSX_API_INVALID_STATE,
            "start_recording: attach and initialize a stage before starting a late recording");
    case omni::physx::OmniPvdRecordingResult::eNotSupported:
        return set_error(OVPHYSX_API_NOT_IMPLEMENTED, "start_recording: OmniPVD recording is unsupported on this platform");
    case omni::physx::OmniPvdRecordingResult::eNotCapable:
        return set_error(
            OVPHYSX_API_INVALID_STATE,
            "start_recording: set OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE=true before creating the first instance");
    case omni::physx::OmniPvdRecordingResult::eError:
    default:
        return set_error(OVPHYSX_API_ERROR, "start_recording: failed to open the destination or start sampling");
    }
}

OVPHYSX_API ovphysx_result_t ovphysx_ovd_stop_recording(ovphysx_instance_handle_t handle)
{
    OVPHYSX_API_PROFILE_ZONE(ovd_stop_recording);
    CompanionTransitionGuard companionTransition(companionRuntime());
    if (!companionTransition)
        return set_error(OVPHYSX_API_INVALID_STATE, "Companion runtime operation is executing; retry transition");
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "stop_recording: invalid handle");

    std::unique_lock<std::mutex> recordingLock(g_gpuAttachMutex, std::defer_lock);
    const ovphysx_api_status_t waitStatus = acquire_shared_runtime_safe_point(handle, recordingLock);
    if (waitStatus != OVPHYSX_API_SUCCESS)
        return set_error(waitStatus, "stop_recording: failed to reach a shared-runtime safe point");
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
    CompanionTransitionGuard companionTransition(companionRuntime());
    if (!companionTransition)
        return set_error(OVPHYSX_API_INVALID_STATE, "Companion runtime operation is executing; retry transition");
    if (!out_is_recording)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_recording: out_is_recording is null");
    *out_is_recording = false;

    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_recording: invalid handle");
    omni::physx::IPhysxSimulation* physxSim =
        instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->getOmniPvdRecordingStatus)
        return set_error(OVPHYSX_API_ERROR, "is_recording: OmniPVD runtime API is unavailable");

    std::unique_lock<std::mutex> recordingLock(g_gpuAttachMutex, std::defer_lock);
    const ovphysx_api_status_t waitStatus = acquire_shared_runtime_safe_point(handle, recordingLock);
    if (waitStatus != OVPHYSX_API_SUCCESS)
        return set_error(waitStatus, "is_recording: failed to reach a shared-runtime safe point");
    bool recording = false;
    const ovphysx_api_status_t status =
        checkRuntimeResult(physxSim->getOmniPvdRecordingStatus(&recording), "is_recording");
    if (status != OVPHYSX_API_SUCCESS)
        return set_error(status, "is_recording: runtime status query failed");
    *out_is_recording = g_omniPvdRecordingOwner == handle && recording;
    return success();
}


// Kept as real exports for binaries built against the pre-migration 0.6 ABI.
OVPHYSX_API ovphysx_result_t ovphysx_start_recording(
    ovphysx_instance_handle_t handle, const ovphysx_omnipvd_destination_t* destination)
{
    OVPHYSX_API_PROFILE_ZONE(start_recording);
    return ovphysx_ovd_start_recording(handle, destination);
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

OVPHYSX_API ovphysx_result_t ovphysx_shutdown(void) {
    OVPHYSX_API_PROFILE_ZONE(shutdown);
    if (ovphysx::isInLogCallback())
        return set_error(OVPHYSX_API_ERROR, "ovphysx_shutdown cannot be called from within a log callback");

    // Companion leases must not be invalidated by process shutdown. Reserve
    // admission while checking so a new session cannot slip past this guard.
    std::unique_lock<std::shared_mutex> companionLock(g_instances_mutex);
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
    // explicit destruction. Further work on them is unsupported. Successful
    // shutdown still disables and drains the application log callback.
    bool hasLiveInstances = false;
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        hasLiveInstances = !g_instances.empty();
    }
    if (hasLiveInstances)
        return ovphysx::shutdownLogCallback();

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
    // a framework release hook removes the cache after its datastore stops at exit.
    omni::physx::runtime::shutdown();
    ovphysx::shutdownCarbonite();
    {
        std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
        g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;
    }

    g_runtimeConfigStarted.store(false, std::memory_order_release);

    return success();
}


OVPHYSX_API ovphysx_enqueue_result_t ovphysx_step(ovphysx_instance_handle_t handle,
                                                          float step_dt) {
    OVPHYSX_API_PROFILE_ZONE(step);
    TaskContext admission;
    try {
        // Reject invalid dt before admission and sim-time updates.
        if (step_dt < 0.0f || !std::isfinite(step_dt))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");

        // Reject a stage-less handle before reaching simulate(). ensure_physics_attached()
        // treats "no stage" as success, and IPhysxSimulation is a process-wide singleton,
        // so simulate() would silently advance whatever other handle's stage is attached
        // while only this handle's first_step_done/warmup_done get set. That would let
        // clone() on the real owner pass its after-step guard. ovphysx_step_sync()
        // has the same guard.
        {
            std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
            InstanceData* instance = get_instance_ptr(handle);
            if (!instance)
                return set_enqueue_error(OVPHYSX_API_ERROR, "Invalid handle");
            const bool hasPhysicsStage = instance->attachHandle != omni::physics::tensors::kNoAttach;
            if (!hasPhysicsStage)
                return set_enqueue_error(OVPHYSX_API_ERROR, "No stage attached");
        }

        // Finish earlier execution before reading this step's start time.
        float current_time = 0.0f;
        {
            std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
            InstanceData* instance = get_instance_ptr(handle);
            if (instance) {
                if (instance->pending_simulation.active) {
                    map_lock.unlock();
                    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
                    if (wait_status != OVPHYSX_API_SUCCESS)
                        return set_enqueue_error(wait_status, "Failed to complete pending operations before step");
                    map_lock.lock();
                    instance = get_instance_ptr(handle);
                }
                if (instance) {
                    current_time = instance->sim_time;
                }
            }
        }

        // Guard the counter advance against float overflow: even with finite
        // inputs, current_time + step_dt can round up to +inf, which would then
        // latch into the counter and poison every subsequent step.
        const float next_time = current_time + step_dt;
        if (!std::isfinite(next_time))
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "sim_time would overflow (current_time + step_dt is not finite)");

        admission.create_task(handle);
        ovphysx_api_status_t status = omni_sdk_physx_simulate_instance(handle, step_dt, current_time, admission.receipt.op_index);

        // A dispatched step advances simulation state even when it reports an error.
        {
            std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
            InstanceData* instance = get_instance_ptr(handle);
            if (instance && instance->pending_simulation.active &&
                instance->pending_simulation.task_index == admission.receipt.op_index) {
                // Cache last dt for contact reads (force = impulse / dt). A zero-dt step
                // simulates nothing (physXUpdateNonRenderDispatch() guards on
                // elapsedSecs > 0), so the impulses of the last positive-dt step are still
                // what a contact read sees: keep that step's dt as the divisor rather than
                // overwriting it, which would scale the force by 1/dt. This mirrors
                // PhysXScene::mCurrentTimeStep, which a zero-dt simulate() likewise leaves
                // unchanged and which the CONTACT_SENSOR read divides by. The 1.0f
                // initializer still covers reads before any step has run.
                if (step_dt > 0.0f)
                    instance->last_step_dt = step_dt;
                // Advance the internal counter once the step is enqueued. For the async
                // path this is dispatch success, not fetch completion, so the counter
                // reflects the step that was issued.
                instance->sim_time = next_time;
                // Tensor reads reuse this step's initialized simulation state.
                if (instance->attachHandle != omni::physics::tensors::kNoAttach) {
                    instance->warmup_done.store(true, std::memory_order_release);
                    instance->warmup_attach_handle.store(instance->attachHandle, std::memory_order_release);
                }
            }
        }

        if (status != OVPHYSX_API_SUCCESS) {
            const ovphysx_string_t error = ovphysx_get_last_error();
            return admission.fail(error.length ? error.ptr : "Step execution failed");
        }

        success();
        return admission.receipt;
    } catch (const std::exception& error) {
        return admission.fail(error.what());
    } catch (...) {
        return admission.fail("Unknown exception during step");
    }
}

// Synchronous step+wait that does not create a public context. Equivalent to
// ovphysx_step() followed by wait_task(). Earlier tasks are drained without
// collecting their outcomes; the synchronous step needs no task record.
OVPHYSX_API ovphysx_result_t ovphysx_step_sync(ovphysx_instance_handle_t handle,
                                                float step_dt) {
    OVPHYSX_API_PROFILE_ZONE(step_sync);
    if (step_dt < 0.0f || !std::isfinite(step_dt)) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");
    }

    // Acquire interfaces and instance pointer once.
    const ovphysx_api_status_t drain_status = omni_sdk_physx_wait_all_pending_internal(handle);
    if (drain_status != OVPHYSX_API_SUCCESS)
        return set_error(drain_status == OVPHYSX_API_NOT_FOUND ? OVPHYSX_API_ERROR : drain_status,
                         "Failed to complete pending execution before synchronous step");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);

    InstanceData* instance = get_instance_ptr(handle);
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

    // Release map lock while waiting for fetchResults() to avoid holding it during the
    // blocking call.
    map_lock.unlock();

    // Blocks until the step results are ready.
    omni::physx::RuntimeErrorScope runtimeError;
    if (checkRuntimeResult(physxSim->fetchResults(instance->attachHandle), "warmup fetchResults") != OVPHYSX_API_SUCCESS)
        return set_error(OVPHYSX_API_ERROR, "warmup: runtime rejected fetchResults");
    if (runtimeError.message()[0])
        return set_error(OVPHYSX_API_ERROR, runtimeError.message());

    // Re-acquire to post-process.
    map_lock.lock();
    instance = get_instance_ptr(handle);
    if (!instance) {
        return set_error(OVPHYSX_API_ERROR, "Handle invalidated during fetchResults");
    }

    // ovphysx does not write results back to ovstage: state is read out via the
    // tensor-binding API and the application writes back to the Stage as needed.

    // Match ovphysx_step(): contact reads convert impulse to force with the dt
    // from the most recent step that actually simulated. A zero-dt step simulates
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
        // consult g_globalConfigValues, so this branch and the cache write
        // below only apply to the remaining, settings-backed keys.
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
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    const char* entryError = nullptr;
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
        // Samples and the Python default start at 1.
        if (read_ordinal == 0) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "attach_ovstage: read_ordinal must be a caller-owned sealed ordinal; 0 is reserved");
        }

        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        if (!instanceShared) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "attach_ovstage: invalid handle");
        }

        const ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
        if (wait_status != OVPHYSX_API_SUCCESS) {
            return set_enqueue_error(wait_status, "Failed to complete pending operations before attach_ovstage");
        }

        // Per-instance API contract: the caller must serialize calls on a single
        // handle (see AGENTS.md / the public header @note). This already-attached
        // check and the attach below are intentionally not locked against concurrent
        // foreground callers on the same instance.
        if (instanceShared->attachHandle != omni::physics::tensors::kNoAttach) {
            return set_enqueue_error(OVPHYSX_API_ERROR,
                             "attach_ovstage: a stage is already attached; detach before attaching ovstage");
        }

        omni::physx::IPhysxSimulation* physxSim =
            instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
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

        admission.create_task(handle);
        instanceShared->ovstage_attach_payload.instance = stage;
        instanceShared->ovstage_attach_payload.dict = nullptr;
        // Production attaches consume ovstage alone; no USD stage-cache identity is needed.
        instanceShared->ovstage_attach_payload.usdStageId = 0;

        // Creation-time replicator env-ids: have the runtime assign environment ids at parse
        // (GPU dynamics + broadphase only) so a later clone() finds the source collision-isolated
        // from co-located copies. Ids can only be set on objects outside a scene, so this must run
        // before the parse. Follows /ovphysx/clone/useEnvIds (default on).
        if (carb::Framework* fw = carb::getFramework()) {
            if (carb::settings::ISettings* settings = fw->tryAcquireInterface<carb::settings::ISettings>()) {
                settings->setDefaultBool("/ovphysx/clone/useEnvIds", true);
                settings->setBool(omni::physx::kSettingReplicatorEnvIdsOnAttach,
                                  settings->getAsBool("/ovphysx/clone/useEnvIds"));
            }
        }

        // GPU selection is process-global and PhysX consumes it synchronously while
        // attachOvstage creates the first GPU scene, so the setting and the attach
        // stay in one transaction. The caller owns the sealed read ordinal, and the
        // initial scene parse reads at it.
        omni::physx::RuntimeErrorScope runtimeError;
        omni::physics::AttachHandle attachHandle = omni::physics::kNoAttach;
        bool attached = false;
        {
            std::lock_guard<std::mutex> attachLock(g_gpuAttachMutex);
            applyAttachTimeGpuSelection(*instanceShared, carb::getFramework(), "attachOvstage");
            bool busy = false;
            attachHandle = omni::physx::runtime::attachOvstageWithAdmission(
                &instanceShared->ovstage_attach_payload, read_ordinal, {}, /*startConfiguredRecording=*/false, busy);
            attached = attachHandle != omni::physics::kNoAttach;
        }
        if (!attached) {
            instanceShared->ovstage_attach_payload = OvstageAttachPayload{};
            set_runtime_error("attach_ovstage: runtime attachment failed", runtimeError.message());
            return admission.fail(ovphysx_get_last_error().ptr);
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
            instanceShared->attachedStageId = stageId;
            instanceShared->attachHandle = attachHandle;
            instanceShared->visualizationScopeDropped = false;
            instanceShared->ovstage_attached = true;
        }
        omni::physx::ovxDebugRenderNoteAttach(handle, attachHandle, 0);
        if (!ovphysx_object_change_attach_instance(handle, attachHandle))
        {
            detach_ovstage_internal(handle, nullptr);
            return admission.fail("attach_ovstage: failed to bind object-change subscriptions");
        }
        // Physics is attached whenever the attach succeeded. A zero stage id only
        // means there is no backing USD stage.
        instanceShared->resetStageFlags(attachHandle != omni::physics::tensors::kNoAttach);
        if (stageId != 0) {
            // Genuinely about the USD stage object, so it stays keyed by stage id.
            registerStageLifecycleEntry(stageId);
        }
        return admission.complete();
    } catch (const std::exception& error) {
        return admission.fail(error.what());
    } catch (...) {
        return admission.fail("Unknown exception during attach_ovstage");
    }
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_update_from_ovstage(ovphysx_instance_handle_t handle,
                                                        ovstage_ordinal_range_t range) {
    OVPHYSX_API_PROFILE_ZONE(update_from_ovstage);
    TaskContext admission;
    try {
        // ovstage's range: [start_ordinal, end_ordinal] when has_start_ordinal, else the
        // single end_ordinal. The runtime drain takes a closed [from, to].
        const uint64_t from_ordinal = range.has_start_ordinal ? range.start_ordinal : range.end_ordinal;
        const uint64_t to_ordinal = range.end_ordinal;
        if (from_ordinal > to_ordinal) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "update_from_ovstage: range.start_ordinal must be <= range.end_ordinal");
        }

        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        if (!instanceShared) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "update_from_ovstage: invalid handle");
        }

        const ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
        if (wait_status != OVPHYSX_API_SUCCESS) {
            return set_enqueue_error(wait_status, "Failed to complete pending operations before update_from_ovstage");
        }

        if (!instanceShared->ovstage_attached) {
            return set_enqueue_error(OVPHYSX_API_ERROR, "update_from_ovstage: no ovstage is attached");
        }

        omni::physx::IPhysxSimulation* physxSim =
            instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
        if (!physxSim || !physxSim->updateFromOvStage) {
            return set_enqueue_error(OVPHYSX_API_ERROR,
                             "update_from_ovstage: IPhysxSimulation::updateFromOvStage is unavailable");
        }

        admission.create_task(handle);
        omni::physx::RuntimeErrorScope runtimeError;
        const bool updated = physxSim->updateFromOvStage(instanceShared->attachHandle, from_ordinal, to_ordinal);
        if (!updated) {
            set_runtime_error("update_from_ovstage: IPhysxSimulation::updateFromOvStage failed",
                              runtimeError.message());
            return admission.fail(ovphysx_get_last_error().ptr);
        }

        return admission.complete();
    } catch (const std::exception& error) {
        return admission.fail(error.what());
    } catch (...) {
        return admission.fail("Unknown exception during update_from_ovstage");
    }
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_clone(ovphysx_instance_handle_t handle,
                                                   ovphysx_string_t source_path_in_usd,
                                                   ovphysx_string_t* target_paths,
                                                   uint32_t num_target_paths,
                                                   const float* anchor_transforms,
                                                   const uint32_t* env_ids) {
    OVPHYSX_API_PROFILE_ZONE(clone);
    TaskContext admission;
    try {
        // Clone the source subtree to the target paths via the PhysX SDK replicator
        // (IPhysxSimulation::cloneEnvironments). Only path strings and a flat
        // [num_target_paths * 7] transform array cross the C ABI. anchor_transforms[i] is
        // the world pose of target_paths[i] (NULL co-locates on the source), and env_ids[i]
        // identifies copy i's logical environment (NULL means per-call numbering).

        // Argument and prerequisite failures reject admission without a receipt.
        if (!source_path_in_usd.ptr || source_path_in_usd.length == 0 || !target_paths || num_target_paths == 0) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     "clone: source_path_in_usd must be non-empty, target_paths non-null, "
                                     "and num_target_paths > 0");
        }

        ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
        if (wait_status != OVPHYSX_API_SUCCESS) {
            return set_enqueue_error(wait_status, wait_status == OVPHYSX_API_NOT_FOUND ?
                "Invalid instance handle" : "Failed to complete pending operations before clone");
        }

        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        if (!instanceShared) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "clone: invalid handle");
        }
        // ovphysx attaches only via ovstage, so cloning runs on the ovstage attach. (The underlying
        // runtime seam is attach-agnostic, but that path is not exposed here.)
        if (!instanceShared->ovstage_attached) {
            return set_enqueue_error(OVPHYSX_API_ERROR,
                                     "clone: no ovstage is attached (call ovphysx_attach_ovstage first)");
        }

        // clone() must be called before warmup() or the first step() in all modes.
        // On GPU, warmup allocates DirectGPU buffers sized by actor count, and cloning
        // after that reallocates and corrupts simulation state. CPU enforces the same
        // ordering so code validated in CPU mode behaves the same on GPU.
        if (instanceShared->warmup_done.load(std::memory_order_acquire) ||
            instanceShared->first_step_done.load(std::memory_order_acquire)) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     "clone() must be called before warmup() and the first step(). "
                                     "Complete detach_ovstage(), then reattach the source stage, "
                                     "to re-clone after warmup.");
        }

        omni::physx::IPhysxSimulation* physxSim =
            instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
        if (!physxSim || !physxSim->cloneEnvironments) {
            return set_enqueue_error(OVPHYSX_API_ERROR, "clone: IPhysxSimulation::cloneEnvironments is unavailable");
        }

        // env-id cross-environment collision filtering is a per-process setting (default on).
        // With explicit transforms the copies are already physically separated, so it is an
        // optional add-on.
        bool useEnvIds = true;
        if (carb::Framework* fw = carb::getFramework()) {
            if (carb::settings::ISettings* settings = fw->tryAcquireInterface<carb::settings::ISettings>()) {
                settings->setDefaultBool("/ovphysx/clone/useEnvIds", true);
                useEnvIds = settings->getAsBool("/ovphysx/clone/useEnvIds");
            }
        }

        // Views are not guaranteed null-terminated and the runtime entry takes C strings, so
        // build a null-terminated source path and a stable array of target C-string pointers.
        const std::string source_path(source_path_in_usd.ptr, source_path_in_usd.length);
        if (source_path.find('\0') != std::string::npos) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                     "clone: source_path_in_usd must not contain an embedded NUL");
        }
        std::vector<std::string> targetStorage;
        std::vector<const char*> targetPtrs;
        targetStorage.reserve(num_target_paths);
        targetPtrs.reserve(num_target_paths);
        // Per-target validation (each target valid, unique, not already existing). Enforced
        // C-first so a bad target cannot create a duplicate PhysX object under an existing path.
        // Invalid target arguments reject admission before execution.
        std::unordered_set<std::string> batchSeen;
        batchSeen.reserve(num_target_paths);
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
            if (target == source_path) {
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                         "clone: target path must differ from the source path");
            }
            if (!batchSeen.insert(target).second) {
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                         "clone: duplicate target path in the same clone() call");
            }
            if (instanceShared->cloned_target_paths.count(target) != 0) {
                return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                         "clone: target path was already cloned on this attach");
            }
            targetStorage.push_back(std::move(target));
            targetPtrs.push_back(targetStorage.back().c_str());
        }

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
        }

        // Reserve target bookkeeping and task failure storage before cloning changes
        // simulation state. Publishing the successful target set is allocation-free.
        instanceShared->cloned_target_paths.reserve(instanceShared->cloned_target_paths.size() + batchSeen.size());
        admission.create_task(handle);
        omni::physx::RuntimeErrorScope runtimeError;
        const bool cloned = physxSim->cloneEnvironments(instanceShared->attachHandle, source_path.c_str(), targetPtrs.data(),
            num_target_paths, anchor_transforms, env_ids, useEnvIds);
        if (!cloned) {
            return admission.fail(runtimeError.message()[0] ? runtimeError.message() :
                "cloneEnvironments failed: invalid source, occupied target, or missing backing stage");
        }
        instanceShared->cloned_target_paths.merge(batchSeen);

        // A tensor backend created before clone() cached the pre-clone actor population and
        // buffer sizes, so invalidate it after a successful clone. Keyed by attach handle
        // rather than stage id, because a stageless attach would otherwise keep pre-clone
        // views live against a changed population.
        if (instanceShared->attachHandle != omni::physics::tensors::kNoAttach) {
            if (omni::physics::tensors::TensorApi* tensorApi =
                    omni::physx::runtime::tryGetTensorApiInterface()) {
                if (tensorApi->resetStage)
                    tensorApi->resetStage(instanceShared->attachHandle);
            }
        }

        return admission.complete();
    } catch (const std::exception& error) {
        return admission.fail(error.what());
    } catch (...) {
        return admission.fail("Unknown exception during clone");
    }
}

static ovphysx_result_t detach_ovstage_internal(ovphysx_instance_handle_t handle, TaskContext* admission) {
    std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
    if (!instanceShared) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "detach_ovstage: invalid handle");
    }
    CompanionTransitionGuard companionTransition(companionRuntime());
    if (!companionTransition)
        return set_error(OVPHYSX_API_INVALID_STATE, "Companion runtime operation is executing; retry transition");
    if (companionRuntime().hasPendingPreviews())
    {
        std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
        if (g_omniPvdRecordingOwner != OVPHYSX_INVALID_HANDLE)
            return set_error(OVPHYSX_API_INVALID_STATE, "Fetch companion preview results before recording detach");
    }
    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before detach_ovstage");
    }

    if (admission)
        admission->create_task(handle);

    const bool hadOvstage = instanceShared->ovstage_attached;

    if (hadOvstage) {
        const int64_t stageId = instanceShared->attachedStageId;
        const omni::physics::tensors::AttachHandle attachHandle = instanceShared->attachHandle;

        // ovx_primpath_t handles are owned by the attached Stage's path
        // dictionary. Drop this attachment's visualization scope and this
        // instance's stored scope before the runtime detaches (and the caller
        // may destroy that dictionary). A replacement stage must start unscoped.
        instanceShared->visualizationScopeDropped = true;
        clearVisualizationScopeTokens(attachHandle);
        omni::physx::ovxDebugRenderNoteAttach(handle, 0, 1);

        cleanup_output_writes(*instanceShared);

        // Release SDF views before PhysX/tensor teardown so native ISdfShapeView
        // objects do not outlive the per-stage SimulationBackend data.
        ovphysx_sdf_view_cleanup_instance(instanceShared.get());

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
            instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
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
            instanceShared->attachedStageId = 0;
            instanceShared->attachHandle = omni::physics::tensors::kNoAttach;
            instanceShared->ovstage_attached = false;
        }
        instanceShared->ovstage_attach_payload = OvstageAttachPayload{};
        instanceShared->resetStageFlags();
    }

    return success();
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_detach_ovstage(ovphysx_instance_handle_t handle) {
    OVPHYSX_API_PROFILE_ZONE(detach_ovstage);
    TaskContext admission;
    try {
        const ovphysx_result_t result = detach_ovstage_internal(handle, &admission);
        if (result.status != OVPHYSX_API_SUCCESS) {
            const ovphysx_string_t error = ovphysx_get_last_error();
            if (!admission.receipt.op_index)
                return set_enqueue_error(result.status, error.ptr);
            return admission.fail(error.ptr);
        }
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
    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before find_prims");
    }

    return set_error(OVPHYSX_API_NOT_IMPLEMENTED, "find_prims not yet implemented");
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_prim_list(ovphysx_instance_handle_t handle,
                                                               ovphysx_prim_list_t* prim_list) {
    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
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
// @covers AC-2 AC-3 AC-4 AC-5 AC-7 AC-8
static bool task_is_collected(const InstanceData& instance, ovphysx_op_index_t index)
{
    if (index <= instance.collected_prefix)
        return true;
    auto range = instance.collected_ranges.upper_bound(index);
    return range != instance.collected_ranges.begin() && index <= std::prev(range)->second;
}

static bool wait_output_is_empty(const ovphysx_wait_task_result_t& result)
{
    return !result.error_tasks && result.num_errors == 0 &&
        result.pending_task.status == OVPHYSX_API_SUCCESS &&
        result.pending_task.op_index == 0 && result.pending_task.instance == 0;
}

// Insert at most one interval. Allocate its node before changing existing state;
// merging and prefix compaction then cannot throw.
static void collect_task_range(InstanceData& instance, ovphysx_op_index_t first, ovphysx_op_index_t last)
{
    if (first > last)
        return;
    auto& ranges = instance.collected_ranges;
    auto begin = ranges.lower_bound(first);
    if (begin != ranges.begin() && std::prev(begin)->second >= first - 1)
        --begin;
    auto end = begin;
    while (end != ranges.end() && end->first <= last + 1) {
        first = std::min(first, end->first);
        last = std::max(last, end->second);
        ++end;
    }
    if (first <= instance.collected_prefix + 1) {
        instance.collected_prefix = std::max(instance.collected_prefix, last);
        ranges.erase(begin, end);
    } else if (begin != end && begin->first == first) {
        begin->second = last;
        ranges.erase(std::next(begin), end);
    } else {
        ranges.emplace_hint(begin, first, last);
        ranges.erase(begin, end);
    }
}

static ovphysx_result_t wait_tasks(
    ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task, bool wait_all,
    ovphysx_timeout_t timeout_ns, ovphysx_wait_task_result_t* output)
{
    try {
        // Instance validation precedes receipt and output validation.
        std::shared_ptr<InstanceData> instance = get_instance(handle);
        if (!instance)
            return set_error(OVPHYSX_API_NOT_FOUND, "Invalid instance handle");
        if (companionRuntime().nativeAccessOnThisThread())
            return set_error(OVPHYSX_API_INVALID_STATE, "Cannot wait or collect tasks inside a companion native-access lease");
        if (!output || !wait_output_is_empty(*output))
            return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Wait result must be non-null and zero-initialized");
        const ovphysx_op_index_t cutoff = instance->next_op_index.load() - 1;
        if (!wait_all) {
            if (task.status != OVPHYSX_API_SUCCESS || !task.op_index || task.instance != handle)
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Task is rejected, zero, or belongs to another instance");
            if (task.op_index > cutoff || task_is_collected(*instance, task.op_index))
                return set_error(OVPHYSX_API_NOT_FOUND, "Task was never issued or was already collected");
        }

        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

        ovphysx_op_index_t pending = 0;
        const auto& simulation = instance->pending_simulation;
        if (simulation.active && simulation.task_index &&
            (wait_all ? simulation.task_index <= cutoff : simulation.task_index == task.op_index)) {
            const ovphysx_op_index_t index = simulation.task_index;
            if (finish_pending_simulation(*instance, timeout_ns, start) == OVPHYSX_API_TIMEOUT)
                pending = index;
        }

        const ovphysx_op_index_t first = wait_all ? instance->collected_prefix + 1 : task.op_index;
        const ovphysx_op_index_t last = wait_all ? (pending ? pending - 1 : cutoff)
                                               : (pending ? task.op_index - 1 : task.op_index);
        std::shared_ptr<TaskWaitStorage> storage;
        if (first <= last) {
            for (auto it = instance->task_errors.lower_bound(first);
                 it != instance->task_errors.end() && it->first <= last; ++it) {
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
        if (storage) {
            owner = std::make_unique<std::shared_ptr<TaskWaitStorage>>(storage);
            std::vector<std::weak_ptr<TaskWaitStorage>>& retained = instance->retained_wait_results;
            retained.erase(std::remove_if(retained.begin(), retained.end(),
                [](const std::weak_ptr<TaskWaitStorage>& result) { return result.expired(); }), retained.end());
            retained.reserve(retained.size() + 1);
            retained.emplace_back(storage);
        }

        collect_task_range(*instance, first, last);
        // The live result owns the collected messages; the instance registry is weak.
        // Everything after acknowledgement is allocation-free.
        if (storage) {
            for (size_t index = 0; index < storage->error_count(); ++index) {
                const auto error = instance->task_errors.find(storage->error_tasks()[index].op_index);
                storage->errors[index].swap(error->second);
                instance->task_errors.erase(error);
            }
            storage->set_owner(owner.release());
            output->error_tasks = storage->error_tasks();
            output->num_errors = storage->error_count();
        }
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
