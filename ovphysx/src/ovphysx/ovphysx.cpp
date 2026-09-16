// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
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
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-CAPI-ATTACH-OWNER-001
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-CAPI-NVTX-001
 * @covers AC-1 AC-2 AC-3 AC-5
 *
 * @implements REQ-CAPI-OVSTAGE-SCHEMA-001
 * @covers AC-1 AC-2 AC-3 AC-5
 */

#include "internal/CpuFeatureCheck.h"
#include "LogManager.hpp"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "cuda_shim/CudaShim.h"
#include <omni/physx/PhysXRuntime.h>
#include "UsdSchemaPaths/UsdSchemaPaths.h"
#include "AsyncEventManager/AsyncEventManager.hpp"
#include <ovstage/ovstage_population.h>
using ovphysx::async::async_event_handle_t;
using ovphysx::async::AsyncEventManager;

#include <carb/ClientUtils.h>
#include <carb/logging/Logger.h>
#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>
#include <atomic>
#include <shared_mutex>
#include <cinttypes> // For PRIu64 portable format specifier

// Platform-specific includes (Windows/Linux dynamic loading)
#include "internal/sdk/PlatformIncludes.hpp"
#include "internal/sdk/ovphysxAsyncWait.hpp"
#include "internal/Nvtx.h"

// PhysX simulation interface
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxSettings.h>
// PhysX foundation interface (GPU availability check)
#include <omni/physx/IPhysxFoundation.h>

#include <omni/physics/tensors/TensorApi.h>
#include <omni/core/BuiltIn.h>
#include <omni/core/OmniInit.h>
// Settings for sharing state
#include <carb/settings/ISettings.h>
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
#include <dlfcn.h>
#endif
#include "internal/sdk/ovphysxSDK.hpp"
#include <omni/physics/tensors/ISdfShapeView.h>
// Internal sidecar API + loader (shared between main library and replicator)
#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"
// Global instance map. shared_ptr storage keeps an instance alive for callers
// that still hold it after erase.
std::unordered_map<ovphysx_handle_t, std::shared_ptr<InstanceData>> g_instances;

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

InstanceData::~InstanceData()
{
}

// Serializes the process-global attach-time GPU selector with scene attachment
// so simultaneous attach calls from different handles cannot overwrite it while
// PhysX consumes it.
std::mutex g_gpuAttachMutex;

// The instance handle that currently owns the one live process-wide
// IPhysxSimulation attach (0 == none). IPhysxSimulation is a process-wide
// singleton and beginSimulationAttach() unconditionally tears down whatever
// attach is currently live, so a second instance's attach attempt is rejected
// rather than silently displacing the first. Always read and written while
// holding g_gpuAttachMutex so the ownership check and the guarded
// physxSim->attachOvstage()/detachStage() call happen atomically.
static ovphysx_handle_t g_liveAttachOwner = 0;

// The public owner of the shared runtime's one active OmniPVD stream. This is
// separate from stage ownership: a peer handle may start recording the one
// attached stage. Recording transitions share g_gpuAttachMutex with attach and
// detach so a stream cannot be rebound while its stage is being torn down.
static ovphysx_handle_t g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;

// Process-wide CPU-only mode. Set via ovphysx_set_cpu_mode(true) before any instances
// are created. When true, IPhysxFoundation::setCpuMode(true) is applied at first attach,
// preventing any CUDA driver contact for the process lifetime.
static std::atomic<bool> g_forceCpuMode{false};

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

namespace
{
// True after ovphysx_initialize() succeeds and until ovphysx_shutdown() clears it.
// A second initialize while this is true is rejected.
std::atomic<bool> g_initialized{false};
}

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

// Lazily calls attachStage() for the current stage if not yet done, then runs the
// initial PhysX scene parse (simulate(0,0) + fetchResults) so TensorAPI can discover
// prims. Deferred until the caller has drained ovstage edits. Called from simulate(),
// warmup(), and create_tensor_binding().
ovphysx_api_status_t ovphysx_ensure_physics_attached(ovphysx_handle_t handle)
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
                physxSim->simulate(0.0f, 0.0f);
                physxSim->fetchResults();
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
                physx_ok = physxSim->attachStage(stageId);
            }

            if (!physx_ok) {
                CARB_LOG_ERROR("[PHYSICS] attachStage() failed for stage %" PRId64
                               " (physx=%s)", stageId,
                               physxSim ? "false" : "unavailable");
                return OVPHYSX_API_ERROR;
            }

            instance->physics_attached.store(true, std::memory_order_release);
            if (g_omniPvdRecordingOwner == OVPHYSX_INVALID_HANDLE &&
                physxSim->isOmniPvdRecording && physxSim->isOmniPvdRecording())
            {
                g_omniPvdRecordingOwner = handle;
            }
            CARB_LOG_INFO("[PHYSICS] Lazy attachStage() completed for stage %" PRId64, stageId);
        }
    }

    // PhysX needs a simulate()+fetchResults() cycle to discover articulations,
    // joints, etc. from the attached scene.
    if (!instance->initial_parse_done.load(std::memory_order_acquire)) {
        if (physxSim) {
            try {
                physxSim->simulate(0.0f, 0.0f);
                physxSim->fetchResults();
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
ovphysx_api_status_t omni_sdk_physx_simulate_instance(ovphysx_handle_t handle, float elapsedTime, float currentTime) {
        if (elapsedTime < 0.0f || elapsedTime > 1.0f) {
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

        // Get PhysX interface before locking or creating events
        auto physxSim = instance->carbonite->getPhysxSimulation();
        if (!physxSim) {
            CARB_LOG_ERROR("[PHYSICS SIMULATION] ERROR: PhysX simulation interface not available");
            return OVPHYSX_API_ERROR;
        }

        // Keep map lock held while locking the instance mutex to prevent iterator
        // invalidation during map rehash.
        std::unique_lock<std::mutex> instance_lock(instance->simulationMutex);

        // PhysX simulate() can be called repeatedly, but only one pending event is
        // tracked, so any previous one is completed and cleaned up first.
        if (instance->pendingSimulationEvent != 0) {
            async_event_handle_t event_to_cleanup = instance->pendingSimulationEvent;
            ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, true, 
                "Replaced by new simulation step");
            ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
            instance->pendingSimulationEvent = 0;
        }
        
        // Create async event for this step (internal only, not returned)
        async_event_handle_t event_handle = ovphysx::async::AsyncEventManager::create_event();
        if (event_handle == 0) {
            CARB_LOG_ERROR("[PHYSICS SIMULATION] ERROR: Failed to create async event");
            return OVPHYSX_API_ERROR;
        }

        try {



            physxSim->simulate(elapsedTime, currentTime);

            // Store pending state. fetchResults() runs in sync().
            instance->pendingElapsedTime = elapsedTime;
            instance->pendingCurrentTime = currentTime;
            instance->pendingSimulationEvent = event_handle;
            
            // Return immediately - event stays PENDING until sync() is called
            return OVPHYSX_API_SUCCESS;
            
        } catch (const std::exception& e) {
            CARB_LOG_ERROR("[PHYSICS SIMULATION] EXCEPTION: %s", e.what());
            ovphysx::async::AsyncEventManager::complete_event(event_handle, false, e.what());
            return OVPHYSX_API_ERROR;
        } catch (...) {
            CARB_LOG_ERROR("[PHYSICS SIMULATION] UNKNOWN EXCEPTION occurred");
            ovphysx::async::AsyncEventManager::complete_event(event_handle, false, "Unknown exception");
            return OVPHYSX_API_ERROR;
        }
}

namespace { // reopen anonymous namespace

    // Waits for the pending simulation step to complete.
    static ovphysx_api_status_t omni_sdk_physx_sync(ovphysx_handle_t handle)
    {
        // Pin the instance without holding the global map lock while blocking.
        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        if (!instanceShared)
        {
            CARB_LOG_ERROR("[PHYSICS SYNC] ERROR: Invalid instance handle: %" PRIu64, handle);
            return OVPHYSX_API_ERROR;
        }
        
        std::unique_lock<std::mutex> instance_lock(instanceShared->simulationMutex);

        if (instanceShared->pendingSimulationEvent == 0) {
            return OVPHYSX_API_SUCCESS;
        }

        const bool hasPhysicsStage = instanceShared->attachHandle != omni::physics::tensors::kNoAttach;

        // fetchResults() with no stage attached hangs PhysX. With no stage this is
        // a no-op, not an error.
        if (!hasPhysicsStage) {
            async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
            ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, true);
            ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
            instanceShared->pendingSimulationEvent = 0;

            return OVPHYSX_API_SUCCESS;
        }

        ovphysx_api_status_t result;
        try {
            auto physxSim = instanceShared->carbonite->getPhysxSimulation();
            
            if (!physxSim) {
                CARB_LOG_ERROR("[PHYSICS SYNC] ERROR: PhysX simulation interface not available");
                async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
                ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, false, 
                    "PhysX simulation interface not available");
                ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
                instanceShared->pendingSimulationEvent = 0;
                result = OVPHYSX_API_ERROR;
            } else {
                physxSim->fetchResults();

                // ovphysx never writes results back to the attached ovstage Stage:
                // simulation state is exposed through the read / tensor-binding API
                // and the application owns writing it back to ovstage.

                async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
                ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, true);
                ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
                instanceShared->pendingSimulationEvent = 0;
                
                result = OVPHYSX_API_SUCCESS;
            }
        } catch (const std::exception& e) {
            CARB_LOG_ERROR("[PHYSICS SYNC] EXCEPTION: %s", e.what());
            async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
            ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, false, e.what());
            ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
            instanceShared->pendingSimulationEvent = 0;
            result = OVPHYSX_API_ERROR;
        } catch (...) {
            CARB_LOG_ERROR("[PHYSICS SYNC] UNKNOWN EXCEPTION occurred");
            async_event_handle_t event_to_cleanup = instanceShared->pendingSimulationEvent;
            ovphysx::async::AsyncEventManager::complete_event(event_to_cleanup, false, "Unknown exception");
            ovphysx::async::AsyncEventManager::cleanup_event(event_to_cleanup);
            instanceShared->pendingSimulationEvent = 0;
            result = OVPHYSX_API_ERROR;
        }
        
        return result;
    }
    // ========================================================================
    // Typed config system: enum to Carbonite path lookup tables
    // ========================================================================
    static const char* s_boolKeyPaths[] = {
        "/physics/disableContactProcessing",
        "/physics/collisionConeCustomGeometry",
        "/physics/collisionCylinderCustomGeometry",
        omni::physx::kOmniPvdOutputEnabled,
        omni::physx::kSettingNvtxEnabled,
        omni::physx::kOmniPvdRecordingCapable,
    };
    static_assert(std::size(s_boolKeyPaths) == OVPHYSX_CONFIG_BOOL_COUNT, "s_boolKeyPaths out of sync with enum");

    static const char* s_int32KeyPaths[] = {
        "/physics/numThreads",
        "/physics/sceneMultiGPUMode",
        omni::physx::kOmniPvdTcpPort,
        omni::physx::kOmniPvdTcpTimeoutMs,
        "/physics/ovstageReadPoolMaxMB",
    };
    static_assert(std::size(s_int32KeyPaths) == OVPHYSX_CONFIG_INT32_COUNT, "s_int32KeyPaths out of sync with enum");

    static const char* s_floatKeyPaths[] = {
        nullptr,
    };
    static_assert(OVPHYSX_CONFIG_FLOAT_COUNT == 0, "Update s_floatKeyPaths when float keys are added");

    static const char* s_stringKeyPaths[] = {
        "/persistent/physics/omniPvdOvdRecordingDirectory",
        "/UJITSO/datastore/localCachePath",
        omni::physx::kOmniPvdTransport,
        omni::physx::kOmniPvdTcpAddress,
    };
    static_assert(std::size(s_stringKeyPaths) == OVPHYSX_CONFIG_STRING_COUNT, "s_stringKeyPaths out of sync with enum");

    // Forward declaration (defined below).
    static void applySettingValue(carb::settings::ISettings* settings, const char* key, const char* value);

    template <size_t N>
    static bool configStringEquals(const ovphysx_string_t& value, const char (&expected)[N])
    {
        constexpr size_t expectedLength = N - 1;
        return expected[expectedLength] == '\0' && value.ptr && value.length == expectedLength &&
               std::memcmp(value.ptr, expected, expectedLength) == 0;
    }

    static bool configStringHasEmbeddedNull(const ovphysx_string_t& value)
    {
        return value.ptr && std::memchr(value.ptr, '\0', value.length) != nullptr;
    }

    static ovphysx_api_status_t applyConfigEntry(carb::settings::ISettings* settings,
                                                  const ovphysx_config_entry_t& entry)
    {
        if (!settings) return OVPHYSX_API_ERROR;
        switch (entry.key_type)
        {
        case OVPHYSX_CONFIG_KEY_TYPE_BOOL:
            if (entry.key.bool_key < 0 || entry.key.bool_key >= OVPHYSX_CONFIG_BOOL_COUNT)
                return OVPHYSX_API_INVALID_ARGUMENT;
            settings->setBool(s_boolKeyPaths[entry.key.bool_key], entry.value.bool_value);
            CARB_LOG_INFO("[Config] Set bool %s = %s", s_boolKeyPaths[entry.key.bool_key], entry.value.bool_value ? "true" : "false");
            return OVPHYSX_API_SUCCESS;
        case OVPHYSX_CONFIG_KEY_TYPE_INT32:
            if (entry.key.int32_key < 0 || entry.key.int32_key >= OVPHYSX_CONFIG_INT32_COUNT)
                return OVPHYSX_API_INVALID_ARGUMENT;
            settings->setInt(s_int32KeyPaths[entry.key.int32_key], entry.value.int32_value);
            CARB_LOG_INFO("[Config] Set int %s = %d", s_int32KeyPaths[entry.key.int32_key], entry.value.int32_value);
            return OVPHYSX_API_SUCCESS;
        case OVPHYSX_CONFIG_KEY_TYPE_FLOAT:
            if (entry.key.float_key < 0 || entry.key.float_key >= OVPHYSX_CONFIG_FLOAT_COUNT)
                return OVPHYSX_API_INVALID_ARGUMENT;
            settings->setFloat(s_floatKeyPaths[entry.key.float_key], entry.value.float_value);
            CARB_LOG_INFO("[Config] Set float %s = %g", s_floatKeyPaths[entry.key.float_key], (double)entry.value.float_value);
            return OVPHYSX_API_SUCCESS;
        case OVPHYSX_CONFIG_KEY_TYPE_STRING:
            if (entry.key.string_key < 0 || entry.key.string_key >= OVPHYSX_CONFIG_STRING_COUNT)
                return OVPHYSX_API_INVALID_ARGUMENT;
            {
                if ((!entry.value.string_value.ptr && entry.value.string_value.length != 0) ||
                    ((entry.key.string_key == OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY ||
                      entry.key.string_key == OVPHYSX_CONFIG_OMNIPVD_TRANSPORT ||
                      entry.key.string_key == OVPHYSX_CONFIG_OMNIPVD_TCP_ADDRESS) &&
                     configStringHasEmbeddedNull(entry.value.string_value)))
                    return OVPHYSX_API_INVALID_ARGUMENT;
                std::string val(
                    entry.value.string_value.ptr ? entry.value.string_value.ptr : "",
                    entry.value.string_value.length);
                settings->setString(s_stringKeyPaths[entry.key.string_key], val.c_str());
                CARB_LOG_INFO("[Config] Set string %s = %s", s_stringKeyPaths[entry.key.string_key], val.c_str());
            }
            return OVPHYSX_API_SUCCESS;
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
                    if (key == s_boolKeyPaths[i]) { CARB_LOG_WARN("[Config] Carbonite key '%s' overlaps typed bool key %d; prefer the typed API.", key.c_str(), i); break; }
                for (int i = 0; i < OVPHYSX_CONFIG_INT32_COUNT; ++i)
                    if (key == s_int32KeyPaths[i]) { CARB_LOG_WARN("[Config] Carbonite key '%s' overlaps typed int32 key %d; prefer the typed API.", key.c_str(), i); break; }
                std::string val(entry.value.string_value.ptr, entry.value.string_value.length);
                if (key == omni::physx::kOmniPvdTcpPort || key == omni::physx::kOmniPvdTcpTimeoutMs)
                {
                    int32_t parsedValue = 0;
                    const std::from_chars_result parsed =
                        std::from_chars(val.data(), val.data() + val.size(), parsedValue, 10);
                    if (parsed.ec != std::errc{} || parsed.ptr != val.data() + val.size())
                        return OVPHYSX_API_INVALID_ARGUMENT;
                    settings->setInt(key.c_str(), parsedValue);
                    return OVPHYSX_API_SUCCESS;
                }
                applySettingValue(settings, key.c_str(), val.c_str());
            }
            return OVPHYSX_API_SUCCESS;
        default:
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
    }

    static bool isOmniPvdCreateOnlyEntry(const ovphysx_config_entry_t& entry)
    {
        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_BOOL)
            return entry.key.bool_key == OVPHYSX_CONFIG_OMNIPVD_OUTPUT_ENABLED ||
                   entry.key.bool_key == OVPHYSX_CONFIG_OMNIPVD_RECORDING_CAPABLE;

        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_INT32)
            return entry.key.int32_key == OVPHYSX_CONFIG_OMNIPVD_TCP_PORT ||
                   entry.key.int32_key == OVPHYSX_CONFIG_OMNIPVD_TCP_TIMEOUT_MS;

        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_STRING)
            return entry.key.string_key == OVPHYSX_CONFIG_OMNIPVD_OVD_RECORDING_DIRECTORY ||
                   entry.key.string_key == OVPHYSX_CONFIG_OMNIPVD_TRANSPORT ||
                   entry.key.string_key == OVPHYSX_CONFIG_OMNIPVD_TCP_ADDRESS;

        if (entry.key_type != OVPHYSX_CONFIG_KEY_TYPE_CARBONITE || !entry.key.carbonite_key.ptr)
            return false;

        return configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdOutputEnabled) ||
               configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdRecordingCapable) ||
               configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdOvdRecordingDirectory) ||
               configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTransport) ||
               configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTcpAddress) ||
               configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTcpPort) ||
               configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdTcpTimeoutMs);
    }

    static bool isOmniPvdOutputEnabledEntry(const ovphysx_config_entry_t& entry)
    {
        if (entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_BOOL)
            return entry.key.bool_key == OVPHYSX_CONFIG_OMNIPVD_OUTPUT_ENABLED;

        return entry.key_type == OVPHYSX_CONFIG_KEY_TYPE_CARBONITE &&
               configStringEquals(entry.key.carbonite_key, omni::physx::kOmniPvdOutputEnabled);
    }

    static ovphysx_api_status_t applyCreateConfigEntries(carb::settings::ISettings* settings,
                                                          const ovphysx_config_entry_t* entries,
                                                          uint32_t entryCount)
    {
        const auto applyPass = [&](bool outputEnabledEntries) -> ovphysx_api_status_t
        {
            for (uint32_t i = 0; i < entryCount; ++i)
            {
                if (isOmniPvdOutputEnabledEntry(entries[i]) != outputEnabledEntries)
                    continue;

                const ovphysx_api_status_t status = applyConfigEntry(settings, entries[i]);
                if (status != OVPHYSX_API_SUCCESS)
                    return status;
            }
            return OVPHYSX_API_SUCCESS;
        };

        // A retained runtime reacts synchronously to the output-enabled setting
        // and reads the current recording directory. Apply that trigger only
        // after all peer settings, preserving caller order within each pass.
        const ovphysx_api_status_t peerStatus = applyPass(false);
        return peerStatus == OVPHYSX_API_SUCCESS ? applyPass(true) : peerStatus;
    }

    static ovphysx_api_status_t validateEffectiveOmniPvdStartupConfig(
        carb::settings::ISettings* settings, const char*& errorMessage)
    {
        carb::settings::ScopedRead settingsRead(settings);
        size_t transportLength = 0;
        size_t directoryLength = 0;
        size_t addressLength = 0;
        const char* transport = settings->getStringBuffer(omni::physx::kOmniPvdTransport, &transportLength);
        const char* directory =
            settings->getStringBuffer(omni::physx::kOmniPvdOvdRecordingDirectory, &directoryLength);
        const char* address = settings->getStringBuffer(omni::physx::kOmniPvdTcpAddress, &addressLength);
        omni::physx::OmniPvdDestination destination;
        if (!omni::physx::normalizeOmniPvdDestination(
                transport, transportLength,
                directory, directoryLength,
                address, addressLength,
                settings->getAsInt(omni::physx::kOmniPvdTcpPort),
                settings->getAsInt(omni::physx::kOmniPvdTcpTimeoutMs),
                destination, errorMessage))
            return OVPHYSX_API_INVALID_ARGUMENT;
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

    using ovstage_get_usd_stage_id_fn = int (*)(
        const void* stage,
        uint64_t* out_usd_stage_id);

#if defined(_WIN32)
    static void* resolve_ovstage_symbol(const char* symbol)
    {
        static const char* const candidates[] = { "ovstage.dll", "libovstage.dll" };
        for (const char* candidate : candidates)
        {
            HMODULE module = GetModuleHandleA(candidate);
            if (module)
            {
                FARPROC proc = GetProcAddress(module, symbol);
                if (proc)
                    return reinterpret_cast<void*>(proc);
            }
        }
        return nullptr;
    }
#else
    static void* resolve_ovstage_symbol(const char* symbol)
    {
        static const char* const candidates[] = { "libovstage.so", "libovstage.so.1", "libovstage.so.0", nullptr };
        for (const char* const* candidate = candidates; *candidate != nullptr; ++candidate)
        {
            void* module = dlopen(*candidate, RTLD_NOLOAD | RTLD_NOW);
            if (module)
            {
                void* proc = dlsym(module, symbol);
                // RTLD_NOLOAD still bumps the refcount on a match. Balance it so repeated
                // lookups do not pin libovstage past unload. The library stays mapped via
                // its real owner, so proc remains valid.
                dlclose(module);
                if (proc)
                    return proc;
            }
        }
        return dlsym(RTLD_DEFAULT, symbol);
    }
#endif

    static uint64_t backing_ovstage_usd_stage_id_or_default(const void* stage)
    {
        if (!stage)
            return 0;

        void* proc = resolve_ovstage_symbol("ovstage_get_usd_stage_id");
        if (!proc)
            return 0;

        ovstage_get_usd_stage_id_fn get_usd_stage_id =
            reinterpret_cast<ovstage_get_usd_stage_id_fn>(proc);
        uint64_t usd_stage_id = 0;
        int result = get_usd_stage_id(stage, &usd_stage_id);
        if (result != 0)
            return 0;

        return usd_stage_id;
    }

    using ovstage_population_register_usd_schemas_fn = int (*)(const ovx_string_t* paths, size_t path_count);
    using ovstage_population_get_last_error_fn = ovx_string_t (*)(void);

    // Attach-time gate on the application's schema registration. ovstage treats
    // re-registering a family as a no-op and reports a family registered after
    // population already read the schema definitions as OVSTAGE_ERROR_OP_FAILED.
    // The attached stage was populated before this call, so the probe tells
    // "registered in time" from "never registered" without changing the registry.
    // Without the registration every Physx* API is dropped from the population and
    // the scene simulates with schema defaults the asset authored against.
    // Carbonite setting (create-time config entry): false turns a failed schema
    // registration probe at attach into a warning instead of a refused attach.
    static const char* const kSettingRequireSchemaRegistration = "/ovphysx/schemas/requireRegistration";

    static const char* const kPhysxSchemasLateMessage =
        "attach_ovstage: the PhysX USD schemas were not registered with ovstage before the first "
        "population in this process, so every Physx* API and attribute was dropped from the populated "
        "stage. Register them before populating: pass ovphysx_get_codeless_schema_root() to "
        "ovstage_population_register_usd_schemas() (Python: "
        "ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])).";

    // Set once the probe below has seen a late registration. The late call itself
    // registers the plugins (ovstage cannot take that back), so a second probe in the
    // same process would report OVSTAGE_OK; the USD registry it built without the
    // schemas is process-global and stays broken, so the refusal has to be too.
    static std::atomic<bool> g_physxSchemasRegisteredLate{ false };

    static ovphysx_api_status_t verify_physx_schemas_registered(std::string& error_out)
    {
        if (g_physxSchemasRegisteredLate.load(std::memory_order_acquire))
        {
            error_out = kPhysxSchemasLateMessage;
            return OVPHYSX_API_ERROR;
        }

        std::string rootError;
        const std::string root = omni::sdk::usd_schema_paths::getCodelessSchemaRoot(&rootError);
        if (root.empty())
        {
            error_out = "attach_ovstage: cannot locate the codeless PhysX schemas: " + rootError;
            return OVPHYSX_API_ERROR;
        }

        void* proc = resolve_ovstage_symbol("ovstage_population_register_usd_schemas");
        if (!proc)
        {
            // No registration entry point in the loaded ovstage: nothing to verify against.
            return OVPHYSX_API_SUCCESS;
        }
        const ovx_string_t path{ root.c_str(), root.size() };
        const int status = reinterpret_cast<ovstage_population_register_usd_schemas_fn>(proc)(&path, 1);
        if (status == OVSTAGE_OK)
        {
            return OVPHYSX_API_SUCCESS;
        }

        if (status == OVSTAGE_ERROR_OP_FAILED)
        {
            g_physxSchemasRegisteredLate.store(true, std::memory_order_release);
            error_out = kPhysxSchemasLateMessage;
        }
        else
        {
            error_out = "attach_ovstage: registering the codeless PhysX schemas at '" + root +
                        "' with ovstage failed with status " + std::to_string(status) + ".";
        }
        if (void* lastError = resolve_ovstage_symbol("ovstage_population_get_last_error"))
        {
            const ovx_string_t msg = reinterpret_cast<ovstage_population_get_last_error_fn>(lastError)();
            if (msg.ptr && msg.length)
            {
                error_out += " ovstage: ";
                error_out.append(msg.ptr, msg.length);
            }
        }
        return OVPHYSX_API_ERROR;
    }

    // Waits on a single event with timeout, running fetchResults() first when the
    // event is the pending simulation step. On failure error_out receives the message.
    static ovphysx_api_status_t wait_on_single_event(ovphysx_handle_t handle, 
                                                      ovphysx_op_index_t op_index,
                                                      async_event_handle_t event,
                                                      ovphysx_timeout_t timeout_ns,
                                                      std::string& error_out,
                                                      bool consume_op_index) {
        if (event == 0) {
            return OVPHYSX_API_ERROR;
        }
        
        // Preserve the max-value sentinel as a literal unbounded wait. A
        // clamped finite duration must not accidentally stand in for forever.
        const bool wait_forever = (timeout_ns == OVPHYSX_TIMEOUT_INFINITE);
        const std::chrono::nanoseconds timeout = clamp_timeout_ns(timeout_ns);
        const bool no_wait = (timeout_ns == OVPHYSX_TIMEOUT_POLL);
        const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
        
        bool needsFetchResults = false;
        {
            std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
            InstanceData* instance = get_instance_ptr(handle);
            if (instance) {
                needsFetchResults = (instance->pendingSimulationEvent == event);
            }
        }

        // Simulation completion requires fetchResults(), but fetchResults() is
        // blocking. Poll checkResults() until ready before finalizing so zero
        // and finite waits honor their timeout and leave a pending op tracked.
        if (needsFetchResults) {
            std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
            if (instanceShared) {
                std::unique_lock<std::mutex> instance_lock(instanceShared->simulationMutex);

                omni::physx::IPhysxSimulation* physxSim = instanceShared->carbonite->getPhysxSimulation();

                if (physxSim && instanceShared->attachHandle != omni::physics::tensors::kNoAttach) {
                    try {
                        if (!wait_forever) {
                            const bool results_ready = ovphysx::async::detail::wait_until_simulation_ready(
                                no_wait,
                                timeout,
                                start,
                                [physxSim]() { return physxSim->checkResults(); },
                                []() { return std::chrono::steady_clock::now(); },
                                [](std::chrono::steady_clock::duration duration) {
                                    std::this_thread::sleep_for(duration);
                                });
                            if (!results_ready) {
                                return OVPHYSX_API_TIMEOUT;
                            }
                        }

                        // Finite waits establish readiness first. Infinite waits
                        // preserve the direct blocking fetch used by the hot path.
                        physxSim->fetchResults();

                        AsyncEventManager::complete_event(event, true);
                    } catch (const std::exception& e) {
                        AsyncEventManager::complete_event(event, false, e.what());
                    } catch (...) {
                        AsyncEventManager::complete_event(event, false, "Unknown exception during fetchResults");
                    }
                } else {
                    // No stage attached, complete as success (no-op)
                    AsyncEventManager::complete_event(event, true);
                }

                instanceShared->pendingSimulationEvent = 0;
            }
        }
        
        while (true) {
            async_status_t status = async_poll_event(event);
            if (status == ASYNC_STATUS_COMPLETED || status == ASYNC_STATUS_FAILED) {
                // A terminal observation wins at the deadline: the operation is
                // no longer pending, so consume and report its actual result.
                if (status == ASYNC_STATUS_FAILED) {
                    error_out = AsyncEventManager::get_event_error(event);
                }

                // Operation completed - remove from tracking map iff this wait consumes the op_index.
                if (consume_op_index) {
                    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
                    InstanceData* instance = get_instance_ptr(handle);
                    if (instance) {
                        std::lock_guard<std::mutex> lock(instance->op_tracking_mutex);
                        if (op_index > instance->last_explicitly_consumed_op_index)
                            instance->last_explicitly_consumed_op_index = op_index;
                        instance->op_to_event.erase(op_index);
                    }
                    // Clean up the event to prevent memory leak in long-running processes
                    AsyncEventManager::cleanup_event(event);
                }
                
                return (status == ASYNC_STATUS_COMPLETED) ? OVPHYSX_API_SUCCESS : OVPHYSX_API_ERROR;
            }
            
            if (no_wait) {
                return OVPHYSX_API_TIMEOUT;
            }

            const std::chrono::steady_clock::duration elapsed = std::chrono::steady_clock::now() - start;
            if (!wait_forever && elapsed >= timeout) {
                return OVPHYSX_API_TIMEOUT;
            }
            const std::chrono::steady_clock::duration sleep_duration = wait_forever
                ? std::chrono::microseconds(100)
                : std::min<std::chrono::steady_clock::duration>(
                    std::chrono::microseconds(100), timeout - elapsed);
            std::this_thread::sleep_for(sleep_duration);
        }
    }
    
    // Waits for every pending operation, then consumes them and cleans up their
    // events so long-running loops do not grow the tracking map. A high-water mark
    // lets a later wait_op() observe an operation consumed by internal synchronization.
    static ovphysx_api_status_t wait_for_all_pending_ops(ovphysx_handle_t handle) {
        std::vector<ovphysx_op_index_t> pending_ops = ovphysx::async::get_pending_ops(handle, OVPHYSX_OP_INDEX_ALL);

        if (pending_ops.empty()) {
            return OVPHYSX_API_SUCCESS;
        }

        // Do not consume yet. All ops are waited on first.
        for (ovphysx_op_index_t pending_op : pending_ops) {
            async_event_handle_t event = ovphysx::async::get_event_for_op(handle, pending_op);

            std::string error_msg;
            ovphysx_api_status_t status = wait_on_single_event(
                handle,
                pending_op,
                event,
                OVPHYSX_TIMEOUT_INFINITE,
                error_msg,
                /*consume_op_index=*/false);
            if (status != OVPHYSX_API_SUCCESS) {
                // Leave the failed and subsequent ops in the map so the user can
                // retrieve errors via wait_op().
                return OVPHYSX_API_ERROR;
            }
        }

        // All ops completed successfully - consume them and update watermark
        {
            std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
            InstanceData* instance = get_instance_ptr(handle);
            if (instance) {
                std::lock_guard<std::mutex> lock(instance->op_tracking_mutex);
                ovphysx_op_index_t max_op = 0;
                for (ovphysx_op_index_t op : pending_ops) {
                    auto it = instance->op_to_event.find(op);
                    if (it != instance->op_to_event.end()) {
                        if (op > max_op) max_op = op;
                        AsyncEventManager::cleanup_event(it->second);
                        instance->op_to_event.erase(it);
                    }
                }
                if (max_op > instance->last_internally_synced_op_index) {
                    instance->last_internally_synced_op_index = max_op;
                }
            }
        }

        return OVPHYSX_API_SUCCESS;
    }

    // Recording mutates the one process-wide PhysX/OmniPVD runtime, while an
    // asynchronous step is tracked only by the handle that owns the live stage.
    // Drain both queues without holding the transition mutex, then reacquire it
    // and verify that the drained owner is still the live owner. Public
    // lifecycle calls follow the same-thread contract, so an ownership change is
    // an invalid transition rather than a concurrency case to retry here.
    static ovphysx_api_status_t acquire_shared_runtime_safe_point(
        ovphysx_handle_t caller, std::unique_lock<std::mutex>& transitionLock)
    {
        const ovphysx_api_status_t callerStatus = wait_for_all_pending_ops(caller);

        ovphysx_handle_t attachOwner = OVPHYSX_INVALID_HANDLE;
        {
            std::lock_guard<std::mutex> lock(g_gpuAttachMutex);
            attachOwner = g_liveAttachOwner;
        }

        ovphysx_api_status_t ownerStatus = OVPHYSX_API_SUCCESS;
        if (attachOwner != OVPHYSX_INVALID_HANDLE && attachOwner != caller)
            ownerStatus = wait_for_all_pending_ops(attachOwner);

        transitionLock.lock();
        if (g_liveAttachOwner != attachOwner)
            return OVPHYSX_API_INVALID_STATE;
        return callerStatus != OVPHYSX_API_SUCCESS ? callerStatus : ownerStatus;
    }

    // Must be called while holding g_gpuAttachMutex, after a shared-runtime
    // transition that may have stopped OmniPVD sampling. Preserve the public
    // owner when sampling is still active. Another handle may own the recording.
    static void reconcile_recording_owner_after_runtime_transition(
        omni::physx::IPhysxSimulation* physxSim)
    {
        if (physxSim && physxSim->isOmniPvdRecording && !physxSim->isOmniPvdRecording())
            g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;
    }
}

// Internal C++ function implementation (exposed via ovphysxSDK.hpp)
ovphysx_api_status_t omni_sdk_physx_wait_all_pending_internal(ovphysx_handle_t handle) {
    // Fast path. When all_ops_synced is set nothing is pending, which skips the two
    // mutexes and the vector allocation in wait_for_all_pending_ops.
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (instance && instance->all_ops_synced.load(std::memory_order_acquire))
            return OVPHYSX_API_SUCCESS;
    }
    ovphysx_api_status_t status = wait_for_all_pending_ops(handle);
    if (status == OVPHYSX_API_SUCCESS) {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (instance)
            instance->all_ops_synced.store(true, std::memory_order_release);
    }
    return status;
}

extern "C" {

ovphysx_api_status_t omni_sdk_physx_set_setting(
    ovphysx_handle_t handle,
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
    ovphysx_handle_t handle,
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

ovphysx_api_status_t omni_sdk_physx_unload_usd(ovphysx_handle_t handle)
{
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(handle);
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
                    if (g_liveAttachOwner != OVPHYSX_INVALID_HANDLE && g_liveAttachOwner != handle)
                    {
                        // This legacy unload path must not detach the shared runtime
                        // while another OvPhysX handle owns the live ovstage attach.
                        CARB_LOG_WARN(
                            "[UNLOAD #%u] Stage %" PRId64
                            " - skipping runtime detach owned by instance %" PRIu64,
                            unloadSeq, stageId, g_liveAttachOwner);
                    }
                    else
                    {
                        if (!instance->physics_attached.load(std::memory_order_acquire))
                        {
                            // Pair a quick attach+detach so the PhysX plugin releases
                            // any refs it accumulated during load.
                            applyAttachTimeGpuSelection(
                                *instance, carb::getFramework(), "late attachStage for detach");
                            physxSim->attachStage(stageId);
                            CARB_LOG_INFO(
                                "[UNLOAD #%u] Stage %" PRId64
                                " - late attach for clean detach (physics was deferred)",
                                unloadSeq, stageId);
                        }

                        CARB_LOG_INFO(
                            "[UNLOAD #%u] Detaching PhysX simulation for stage %" PRId64,
                            unloadSeq, stageId);
                        physxSim->detachStage();
                        reconcile_recording_owner_after_runtime_transition(physxSim);
                        if (g_liveAttachOwner == handle)
                            g_liveAttachOwner = OVPHYSX_INVALID_HANDLE;
                        CARB_LOG_INFO(
                            "[UNLOAD #%u] PhysX simulation detached for stage %" PRId64,
                            unloadSeq, stageId);
                    }
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

            instance->attachedStageId = 0;
            instance->attachHandle = omni::physics::tensors::kNoAttach;
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

static void clearVisualizationScopeTokens()
{
    if (OvphysxSidecarSetVizScopeTokensFn clearScope =
            g_sidecarSetVizScopeTokens.load(std::memory_order_acquire))
    {
        (void)clearScope(nullptr, 0u);
    }
}

ovphysx_api_status_t omni_sdk_physx_destroy(ovphysx_handle_t handle)
{
    // Reject an already-absent handle before any process-global teardown. This
    // defensive path lets Python recover from an ambiguous FFI exception
    // without disturbing live peers or their asynchronous state.
    {
        std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
        std::unordered_map<ovphysx_handle_t, std::shared_ptr<InstanceData>>::const_iterator it =
            g_instances.find(handle);
        if (it == g_instances.end() || !it->second)
            return OVPHYSX_API_ERROR;
    }

    // Destruction can finalize a stream owned by a peer of the attached stage, so
    // establish the same shared-runtime safe point as explicit start/stop before
    // any teardown. Failures are ignored so destruction still makes best-effort
    // progress.
    bool recordingOwnedAtDestroy = false;
    {
        std::unique_lock<std::mutex> transitionLock(g_gpuAttachMutex, std::defer_lock);
        (void)acquire_shared_runtime_safe_point(handle, transitionLock);
        recordingOwnedAtDestroy = g_omniPvdRecordingOwner == handle;
    }

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
        const ovphysx_result_t detachResult = ovphysx_detach_ovstage(handle);
        if (detachResult.status != OVPHYSX_API_SUCCESS)
        {
            // Failed pending work can prevent the normal detach path from
            // running. The scope owns dictionary-backed tokens, so clear it
            // before the instance (and potentially the dictionary) disappears.
            clearVisualizationScopeTokens();

            // ovphysx_detach_ovstage() bailed out before its runtime-detach block
            // (for example wait_for_all_pending_ops() failed), so this handle may
            // still hold the process-wide live-attach latch. Force the release here,
            // independently of pending-op success, so destruction never leaves that
            // latch stuck.
            std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
            std::lock_guard<std::mutex> attachLock(g_gpuAttachMutex);
            if (g_liveAttachOwner == handle)
            {
                omni::physx::IPhysxSimulation* physxSim =
                    instanceShared && instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
                if (physxSim && physxSim->detachStage)
                {
                    physxSim->detachStage();
                    reconcile_recording_owner_after_runtime_transition(physxSim);
                }
                g_liveAttachOwner = OVPHYSX_INVALID_HANDLE;
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
            if (physxSim && physxSim->stopOmniPvdRecording && physxSim->isOmniPvdRecording &&
                physxSim->isOmniPvdRecording())
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

    async_cleanup_all_events();

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

            instanceShared->carbonite.reset();
        }
        catch (const std::exception& e)
        {
            CARB_LOG_ERROR("[DESTROY] Exception during component cleanup: %s", e.what());
        }
        catch (...)
        {
            CARB_LOG_ERROR("[DESTROY] Unknown exception during component cleanup");
        }
    }

    omni::physics::tensors::TensorApi* tensorApi = omni::physx::runtime::tryGetTensorApiInterface();
    if (tensorApi)
    {
        // Per-instance fallback when unload did not clear this stage. Unlike the full
        // reset() below this needs no g_createInstanceMutex serialization, because it
        // only touches destroyedAttachHandle's entries and a concurrent create_instance
        // mints a different handle. SimulationBackend::resetStage takes its own lock.
        if (tensorApi->resetStage && destroyedAttachHandle != omni::physics::tensors::kNoAttach &&
            !stageUnloadCompleted)
            tensorApi->resetStage(destroyedAttachHandle);

        // On the last instance a full backend reset releases the remaining resources.
        // reset() clears sim data for all stages, so g_createInstanceMutex is held
        // across the emptiness re-check and the reset() call. create_instance holds
        // the same mutex while inserting into g_instances, so no instance can appear
        // in between. Lock order (create mutex then instances mutex) matches
        // createInstanceInternal().
        if (lastInstance && tensorApi->reset)
        {
            std::lock_guard<std::mutex> createLock(g_createInstanceMutex);
            bool stillLast = false;
            {
                std::shared_lock<std::shared_mutex> recheck(g_instances_mutex);
                stillLast = g_instances.empty();
            }
            if (stillLast)
                tensorApi->reset();
        }
    }

    if (lastInstance)
    {
        CARB_LOG_INFO("[ovphysx] Direct runtime retained until ovphysx_shutdown");
    }

    return OVPHYSX_API_SUCCESS;
}
// ==================== Public API functions ====================

OVPHYSX_API ovphysx_result_t ovphysx_get_codeless_schema_root(ovphysx_string_t* out_root)
{
    if (!out_root)
    {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "ovphysx_get_codeless_schema_root: out_root is null");
    }

    // Thread-local so concurrent callers never observe each other's buffer and
    // tests can point OVPHYSX_LIB at a different layout between calls. The
    // returned view stays valid until this thread calls the function again.
    thread_local std::string s_root;
    std::string error;
    s_root = omni::sdk::usd_schema_paths::getCodelessSchemaRoot(&error);
    out_root->ptr = s_root.c_str();
    out_root->length = s_root.size();
    if (s_root.empty())
    {
        return set_error(OVPHYSX_API_ERROR, error);
    }
    return success();
}

// Main SDK bootstrap entry point.
// It serializes process-global setup, starts Carbonite, applies user config,
// then loads ovphysx's own PhysX plugins. It rejects a process that already
// loaded a PhysX Carbonite stack. ovphysx neither loads nor registers USD here:
// the application owns its USD runtime and registers ovphysx's schemas itself
// (see ovphysx_get_codeless_schema_root).
static ovphysx_result_t createInstanceInternal(const ovphysx_create_args* create_args, ovphysx_handle_t* out_handle) {
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
        isFirstLiveInstance = g_instances.empty();
    }
    if (args->config_entry_count > 0 && !isFirstLiveInstance)
    {
        for (uint32_t i = 0; i < args->config_entry_count; ++i)
        {
            if (isOmniPvdCreateOnlyEntry(args->config_entries[i]))
                return set_error(
                    OVPHYSX_API_ERROR,
                    "OmniPVD creation config cannot be applied while an instance exists");
        }
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
    instanceData->carbonite = std::make_unique<ovphysx::CarboniteLoader>();
    if (!instanceData->carbonite->initialize()) {
        const std::string& loaderError = instanceData->carbonite->getLastError();
        const char* message = loaderError.empty() ? "Failed to initialize Carbonite and load PhysX plugins"
                                                  : loaderError.c_str();
        CARB_LOG_ERROR("%s", message);
        return set_error(OVPHYSX_API_ERROR, message);
    }

    bool reserveOmniPvdStartupOwner = false;

    // Apply user config entries BEFORE loading PhysX plugins. OmniPVD recording
    // is initialized during createPhysics() (triggered by loadPhysxPlugins), so
    // settings like omniPvdOutputEnabled must already be in place.
    {
        carb::Framework* framework = carb::getFramework();
        carb::settings::ISettings* settings =
            framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
        if (settings)
        {
            settings->setDefaultString(omni::physx::kOmniPvdTransport, "file");
            settings->setDefaultString(omni::physx::kOmniPvdTcpAddress, "");
            settings->setDefaultInt(omni::physx::kOmniPvdTcpPort, 0);
            settings->setDefaultInt(omni::physx::kOmniPvdTcpTimeoutMs, 0);
            if (args->config_entry_count > 0)
            {
                CARB_LOG_INFO(
                    "[ovphysx] Applying %u user config entries (pre-plugin-load)",
                    args->config_entry_count);
                const ovphysx_api_status_t applyStatus =
                    applyCreateConfigEntries(settings, args->config_entries, args->config_entry_count);
                if (applyStatus != OVPHYSX_API_SUCCESS)
                    return set_error(applyStatus, "Invalid config entry");
            }
            const char* destinationError = nullptr;
            const ovphysx_api_status_t destinationStatus =
                validateEffectiveOmniPvdStartupConfig(settings, destinationError);
            if (destinationStatus != OVPHYSX_API_SUCCESS)
                return set_error(destinationStatus, destinationError ? destinationError : "Invalid OmniPVD destination");
            reserveOmniPvdStartupOwner = isFirstLiveInstance &&
                settings->getAsBool(omni::physx::kOmniPvdOutputEnabled) &&
                !settings->getAsBool(omni::physx::kOmniPvdIsOVDStage);
        }
        else
        {
            CARB_LOG_WARN("[ovphysx] Warning: ISettings not available, cannot apply user config entries");
        }
    }

    // Resolve NVTX profiling before the PhysX plugins load: the omni.physx runtime
    // reads /physics/nvtxEnabled while creating the PhysX SDK, and this call also
    // writes the OVPHYSX_NVTX environment variable through to that setting.
    ovphysx::nvtx::resolveEnabled();

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
        !isProcessGpuDisabled())
    {
        if (omni::physx::cudaShim::isCudaAvailable())
        {
            const int32_t gpuIndex = requestedOrdinals.empty() ? 0 : requestedOrdinals[0];
            int count = 0;
            const CUresult deviceCountResult = omni::physx::cudaShim::cuDeviceGetCount_(&count);
            const bool haveCount = (deviceCountResult == CUDA_SUCCESS && count > 0);

            // -1 means "PhysX auto-select" and is intentionally left unchecked.
            if (gpuIndex >= 0 && haveCount && gpuIndex >= count)
            {
                char buf[192];
                snprintf(buf, sizeof(buf),
                         "active_cuda_gpus: ordinal %d is out of range (CUDA device count: %d)",
                         gpuIndex, count);
                CARB_LOG_ERROR("[ovphysx] %s", buf);
                return set_error(OVPHYSX_API_INVALID_ARGUMENT, buf);
            }

            // Validate multi-GPU patterns at create time so callers get a fast,
            // clear error rather than a silent single-GPU fallback at attach time.
            if (requestedOrdinals.size() > 1 && haveCount)
            {
                char modeErr[384] = {};
                if (determineMultiGPUMode(requestedOrdinals, count, modeErr, sizeof(modeErr)) < 0)
                {
                    CARB_LOG_ERROR("[ovphysx] %s", modeErr);
                    return set_error(OVPHYSX_API_INVALID_ARGUMENT, modeErr);
                }
            }
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
        // Persist the parsed ordinals only when the caller restricted GPU ordinals.
        // requestedOrdinals defaults to {0} for empty input, so gate on the original
        // string to keep "no active_cuda_gpus" distinct from an explicit "0".
        if (args->active_cuda_gpus.ptr && args->active_cuda_gpus.length > 0) {
            instanceData->active_cuda_ordinals = requestedOrdinals;
        }
    } else {
        instanceData->create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    }

    const ovphysx_handle_t handle = ovphysx::internal::allocateOpaqueObjectHandle();
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
    if (reserveOmniPvdStartupOwner)
    {
        std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
        if (g_omniPvdRecordingOwner == OVPHYSX_INVALID_HANDLE)
            g_omniPvdRecordingOwner = handle;
    }
    *out_handle = handle;

    return success();
}

// Process-wide lifecycle: ovphysx_initialize() / ovphysx_shutdown() / the
// init-check in ovphysx_create_instance().
//
// Per the ovphysx threading contract, process-lifecycle calls are serialized by
// the caller (the Python wrapper uses its lifecycle condition). The g_initialized
// atomic only enforces matched init/shutdown pairing, it is not a lock. These are
// not guarded with g_createInstanceMutex because createInstanceInternal() already
// takes that non-recursive mutex, so checking g_initialized under it would
// self-deadlock.
//
// Initialize performs only CPU capability validation, the atomic lifecycle claim,
// and environment-policy latching. It must not emit logs, invoke callbacks, or
// wait for callback delivery, because the Python wrapper lets concurrent
// constructors wait for this transition. Shutdown can flush and drain callbacks,
// so Python callers racing final shutdown remain fail-fast.
OVPHYSX_API ovphysx_result_t ovphysx_initialize(void)
{
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
    if (!g_initialized.compare_exchange_strong(expected, true, std::memory_order_acq_rel))
    {
        return set_error(OVPHYSX_API_ERROR, "ovphysx_initialize called while already initialized");
    }

    // Sample OVPHYSX_DISABLE_GPU at the documented initialization boundary so a
    // prior observational get_cpu_mode() cannot permanently miss a later setenv.
    latchEnvGpuDisabledFromEnvironment();

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_create_instance(const ovphysx_create_args* create_args, ovphysx_handle_t* out_handle)
{
    if (!g_initialized.load(std::memory_order_acquire))
    {
        if (out_handle)
        {
            *out_handle = OVPHYSX_INVALID_HANDLE;
        }
        return set_error(OVPHYSX_API_ERROR, "ovphysx_create_instance called before ovphysx_initialize");
    }

    return createInstanceInternal(create_args, out_handle);
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_instance(ovphysx_handle_t handle) {
    // No g_createInstanceMutex here. omni_sdk_physx_destroy has its own
    // serialization, and ovphysx_destroy_instance is called from PhysX RAII
    // destructors that can fire on error paths inside createInstanceInternal,
    // which holds that mutex, so taking it here would deadlock.
    ovphysx_api_status_t status = omni_sdk_physx_destroy(handle);
    if (status != OVPHYSX_API_SUCCESS) {
        return {status}; // Don't set error on destroy failure
    }
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_start_recording(
    ovphysx_handle_t handle, const ovphysx_omnipvd_destination_t* destination)
{
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

    if (physxSim->isOmniPvdRecording && physxSim->isOmniPvdRecording())
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
            "start_recording: set omnipvd_recording_capable=true before creating the first instance");
    case omni::physx::OmniPvdRecordingResult::eError:
    default:
        return set_error(OVPHYSX_API_ERROR, "start_recording: failed to open the destination or start sampling");
    }
}

OVPHYSX_API ovphysx_result_t ovphysx_stop_recording(ovphysx_handle_t handle)
{
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
    if (!physxSim->isOmniPvdRecording || !physxSim->isOmniPvdRecording())
        return set_error(OVPHYSX_API_INVALID_STATE, "stop_recording: no recording is active");

    const omni::physx::OmniPvdRecordingResult result = physxSim->stopOmniPvdRecording();
    if (!physxSim->isOmniPvdRecording || !physxSim->isOmniPvdRecording())
        g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;
    if (result == omni::physx::OmniPvdRecordingResult::eSuccess)
        return success();
    if (result == omni::physx::OmniPvdRecordingResult::eInvalidState)
        return set_error(OVPHYSX_API_INVALID_STATE, "stop_recording: no recording is active");
    if (result == omni::physx::OmniPvdRecordingResult::eNotSupported)
        return set_error(OVPHYSX_API_NOT_IMPLEMENTED, "stop_recording: OmniPVD recording is unsupported on this platform");
    return set_error(OVPHYSX_API_ERROR, "stop_recording: failed to finalize the recording");
}

OVPHYSX_API ovphysx_result_t ovphysx_is_recording(
    ovphysx_handle_t handle, bool* out_is_recording)
{
    if (!out_is_recording)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_recording: out_is_recording is null");
    *out_is_recording = false;

    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "is_recording: invalid handle");
    omni::physx::IPhysxSimulation* physxSim =
        instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->isOmniPvdRecording)
        return set_error(OVPHYSX_API_ERROR, "is_recording: OmniPVD runtime API is unavailable");

    std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
    *out_is_recording = g_omniPvdRecordingOwner == handle && physxSim->isOmniPvdRecording();
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_set_cpu_mode(bool cpu_only)
{
    std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
    if (!g_instances.empty())
        return set_error(OVPHYSX_API_ERROR,
            "ovphysx_set_cpu_mode: cannot change CPU mode while instances are active; "
            "destroy all instances first");
    if (!cpu_only && g_forceCpuMode.load(std::memory_order_acquire))
        return set_error(OVPHYSX_API_ERROR,
            "ovphysx_set_cpu_mode: CPU-only mode is sticky once enabled; "
            "cannot revert to GPU mode in this process");
    g_forceCpuMode.store(cpu_only, std::memory_order_release);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_cpu_mode(bool* out_cpu_only)
{
    if (!out_cpu_only)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
            "ovphysx_get_cpu_mode: out_cpu_only must not be null");
    *out_cpu_only = isProcessGpuDisabled();
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_shutdown(void) {
    if (ovphysx::isInLogCallback())
        return set_error(OVPHYSX_API_ERROR, "ovphysx_shutdown cannot be called from within a log callback");

    // Guard: must be called with a matching ovphysx_initialize().
    bool expected = true;
    if (!g_initialized.compare_exchange_strong(expected, false, std::memory_order_acq_rel))
    {
        return set_error(OVPHYSX_API_ERROR, "ovphysx_shutdown called without matching ovphysx_initialize");
    }

    // Allow the next initialize() to re-sample OVPHYSX_DISABLE_GPU. Pre-init
    // get_cpu_mode() between shutdown and re-init reads the env live again.
    clearEnvGpuDisabledLatch();

    // With instances still alive, g_initialized is cleared above so a second
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
    // The Carbonite framework itself is kept resident for its static
    // process-exit hook. Only the direct runtime is drained here.
    omni::physx::runtime::shutdown();
    {
        std::lock_guard<std::mutex> recordingLock(g_gpuAttachMutex);
        g_liveAttachOwner = OVPHYSX_INVALID_HANDLE;
        g_omniPvdRecordingOwner = OVPHYSX_INVALID_HANDLE;
    }

    CARB_LOG_VERBOSE("[ovphysx] Direct runtime shut down; Carbonite framework kept resident until process exit");

    return ovphysx::shutdownLogCallback();
}


OVPHYSX_API ovphysx_enqueue_result_t ovphysx_reset_stage(ovphysx_handle_t handle) {
    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_enqueue_error(wait_status, "Failed to complete pending operations before reset_stage");
    }

    // Gating solely on ovstage_attached is complete. attachedStageId / attachHandle
    // are set and cleared only by ovphysx_attach_ovstage() / ovphysx_detach_ovstage(),
    // always together with ovstage_attached, so no live attach path leaves a handle
    // attached while ovstage_attached is false. The stageId-keyed fallback in
    // omni_sdk_physx_unload_usd() is a defensive safety net for full instance
    // teardown, not a second attach flow.
    bool detach_ovstage = false;
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (!instance) {
            return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid handle");
        }
        detach_ovstage = instance->ovstage_attached;
    }
    if (detach_ovstage) {
        ovphysx_result_t detach_result = ovphysx_detach_ovstage(handle);
        if (detach_result.status != OVPHYSX_API_SUCCESS) {
            return set_enqueue_error(detach_result.status, "Failed to detach ovstage during reset_stage");
        }
    }

    // NOTE: stage teardown deliberately does NOT touch the internal
    // simulation-time counter (mirrors ovrtx_reset_stage). The counter is a
    // private implementation detail advanced only by the step entry points.

    async_event_handle_t event = AsyncEventManager::create_event();
    AsyncEventManager::complete_event(event, true);

    ovphysx_op_index_t op_index = ovphysx::async::register_operation(handle, event);

    return enqueue_success(op_index);
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_step(ovphysx_handle_t handle,
                                                          float step_dt) {
    OVPHYSX_NVTX_ZONE("ovphysx_step");
    // Reject negative / non-finite dt up front so a bad value never advances
    // (and poisons) the internal sim-time counter.
    if (step_dt < 0.0f || !std::isfinite(step_dt))
        return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");

    // Reject a stage-less handle before reaching simulate(). ensure_physics_attached()
    // treats "no stage" as success, and IPhysxSimulation is a process-wide singleton,
    // so simulate() would silently advance whatever other handle's stage is attached
    // while only this handle's first_step_done/warmup_done get set. That would let
    // clone() on the real owner pass its after-step guard. ovphysx_step_sync() and
    // ovphysx_step_n_sync() have the same guard.
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (!instance)
            return set_enqueue_error(OVPHYSX_API_ERROR, "Invalid handle");
        const bool hasPhysicsStage = instance->attachHandle != omni::physics::tensors::kNoAttach;
        if (!hasPhysicsStage)
            return set_enqueue_error(OVPHYSX_API_ERROR, "No stage attached");
    }

    // Wait for pending ops + clear fast-path flag, and read the current
    // sim-time counter (start time of this step) in one lock acquisition.
    float current_time = 0.0f;
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (instance) {
            if (!instance->all_ops_synced.load(std::memory_order_acquire)) {
                map_lock.unlock();
                ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
                if (wait_status != OVPHYSX_API_SUCCESS)
                    return set_enqueue_error(wait_status, "Failed to complete pending operations before step");
                map_lock.lock();
                instance = get_instance_ptr(handle);
            }
            if (instance) {
                instance->all_ops_synced.store(false, std::memory_order_release);
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

    ovphysx_api_status_t status = omni_sdk_physx_simulate_instance(handle, step_dt, current_time);

    if (status != OVPHYSX_API_SUCCESS) {
        return set_enqueue_error(status, "Step failed");
    }

    // Get the pending event, cache dt, advance the counter, mark warmup done.
    async_event_handle_t event = 0;
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (instance) {
            event = instance->pendingSimulationEvent;
            // Cache last dt for contact reads (force = impulse / dt). A zero-dt step
            // produces zero impulses, so substituting 1.0 avoids the division by
            // zero without changing the result.
            instance->last_step_dt = (step_dt > 0.0f) ? step_dt : 1.0f;
            // Advance the internal counter once the step is enqueued. For the async
            // path this is dispatch success, not fetch completion, so the counter
            // reflects the step that was issued.
            instance->sim_time = next_time;
            // Mark warmup as done so clone() can guard against post-step cloning.
            if (instance->attachHandle != omni::physics::tensors::kNoAttach) {
                instance->warmup_done.store(true, std::memory_order_release);
                instance->warmup_attach_handle.store(instance->attachHandle, std::memory_order_release);
            }
            // Mark first-step-done in both CPU and GPU mode so clone()'s after-step
            // guard fires even without an explicit warmup(). Gated on this handle's
            // own attach, so a handle with nothing attached is never marked as
            // having stepped.
            if (instance->attachHandle != omni::physics::tensors::kNoAttach) {
                instance->first_step_done.store(true, std::memory_order_release);
            }
        }
    }

    ovphysx_op_index_t op_index = ovphysx::async::register_operation(handle, event);

    return enqueue_success(op_index);
}

// Synchronous step+wait that bypasses the async event machinery. Equivalent to
// ovphysx_step() followed by wait_op(), with a single lock acquisition and no
// AsyncEventManager overhead.
OVPHYSX_API ovphysx_result_t ovphysx_step_sync(ovphysx_handle_t handle,
                                                float step_dt) {
    OVPHYSX_NVTX_ZONE("ovphysx_step_sync");
    if (step_dt < 0.0f || !std::isfinite(step_dt)) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");
    }

    // Acquire interfaces and instance pointer once.
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
    physxSim->simulate(step_dt, current_time);

    instance->warmup_done.store(true, std::memory_order_release);
    instance->warmup_attach_handle.store(instance->attachHandle, std::memory_order_release);

    // Release map lock while waiting for fetchResults() to avoid holding it during the
    // blocking call.
    map_lock.unlock();

    // Blocks until the step results are ready.
    physxSim->fetchResults();

    // Re-acquire to post-process.
    map_lock.lock();
    instance = get_instance_ptr(handle);
    if (!instance) {
        return set_error(OVPHYSX_API_ERROR, "Handle invalidated during fetchResults");
    }

    // ovphysx does not write results back to ovstage: state is read out via the
    // tensor-binding API and the application writes back to the Stage as needed.

    // Match ovphysx_step(): contact reads convert impulse to force with the dt
    // from the most recent successfully completed step.
    instance->last_step_dt = (step_dt > 0.0f) ? step_dt : 1.0f;

    // Advance the internal counter after a successful step.
    instance->sim_time = next_time;

    // Mark all ops synced (no pending async ops after a sync step).
    instance->all_ops_synced.store(true, std::memory_order_release);

    return success();
}

// Run n_steps consecutive physics steps in a single C call, saving (n_steps-1)
// ctypes round-trips.  Step i simulates at current_time + i * step_dt.
OVPHYSX_API ovphysx_result_t ovphysx_step_n_sync(ovphysx_handle_t handle,
                                                  int32_t n_steps,
                                                  float step_dt) {
    OVPHYSX_NVTX_ZONE("ovphysx_step_n_sync");
    if (n_steps <= 0)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "n_steps must be > 0");
    if (step_dt < 0.0f || !std::isfinite(step_dt))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);

    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "Invalid handle");

    auto physxSim = instance->carbonite->getPhysxSimulation();
    const bool hasPhysicsStage = instance->attachHandle != omni::physics::tensors::kNoAttach;
    if (!physxSim || !hasPhysicsStage)
        return set_error(OVPHYSX_API_ERROR, "No stage attached");

    // Base time for the batch is the internal counter (start of next step).
    // Guard the whole batch against float overflow up front: since step_dt >= 0,
    // base_time + i*step_dt for any i in [0, n_steps] is <= base_time +
    // n_steps*step_dt, so a finite end time implies every per-step time is
    // finite too. A non-finite end would latch into the counter.
    const float base_time = instance->sim_time;
    const float next_time = base_time + static_cast<float>(n_steps) * step_dt;
    if (!std::isfinite(next_time))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "sim_time would overflow (base_time + n_steps*step_dt is not finite)");

    for (int32_t i = 0; i < n_steps; ++i) {
        const float sim_time = base_time + i * step_dt;

        physxSim->simulate(step_dt, sim_time);

        instance->warmup_done.store(true, std::memory_order_release);
        instance->warmup_attach_handle.store(instance->attachHandle, std::memory_order_release);

        map_lock.unlock();
        physxSim->fetchResults();
        map_lock.lock();

        instance = get_instance_ptr(handle);
        if (!instance)
            return set_error(OVPHYSX_API_ERROR, "Handle invalidated during fetchResults");

        // No ovphysx to ovstage write-back. Results are consumed via the read /
        // tensor-binding API and the application owns writing them back to ovstage.
    }

    // The whole batch completed successfully. Every step used the same dt, so
    // publish it once for contact impulse-to-force conversion.
    instance->last_step_dt = (step_dt > 0.0f) ? step_dt : 1.0f;

    // Advance the internal counter by the full batch after success.
    instance->sim_time = next_time;

    instance->all_ops_synced.store(true, std::memory_order_release);
    return success();
}

// ========================================================================
// Typed config API
// ========================================================================

OVPHYSX_API ovphysx_result_t ovphysx_set_global_config(ovphysx_config_entry_t entry) {
    std::unique_lock<std::mutex> createLock(g_createInstanceMutex, std::defer_lock);
    if (isOmniPvdCreateOnlyEntry(entry))
    {
        createLock.lock();
        std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
        if (!g_instances.empty())
            return set_error(OVPHYSX_API_ERROR, "OmniPVD startup config cannot change while an instance exists");
    }
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    ovphysx_api_status_t status = applyConfigEntry(settings, entry);
    if (status != OVPHYSX_API_SUCCESS) return set_error(status, "Invalid config entry");
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_bool(ovphysx_config_bool_t key, bool* out_value) {
    if (!out_value || key < 0 || key >= OVPHYSX_CONFIG_BOOL_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    *out_value = settings->getAsBool(s_boolKeyPaths[key]);
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_int32(ovphysx_config_int32_t key, int32_t* out_value) {
    if (!out_value || key < 0 || key >= OVPHYSX_CONFIG_INT32_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    *out_value = settings->getAsInt(s_int32KeyPaths[key]);
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_float(ovphysx_config_float_t key, float* out_value) {
    if (!out_value || key < 0 || key >= OVPHYSX_CONFIG_FLOAT_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    *out_value = settings->getAsFloat(s_floatKeyPaths[key]);
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_get_global_config_string(ovphysx_config_string_t key, ovphysx_string_t* value_out, size_t* out_required_size) {
    if (!value_out || !value_out->ptr || value_out->length == 0 || !out_required_size ||
        key < 0 || key >= OVPHYSX_CONFIG_STRING_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
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

OVPHYSX_API ovphysx_result_t ovphysx_attach_ovstage(ovphysx_handle_t handle,
                                                     ovstage_instance_t* stage,
                                                     ovstage_ordinal_t read_ordinal) {
    OVPHYSX_NVTX_ZONE("ovphysx_attach_ovstage");
    if (!stage) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "attach_ovstage: stage is null");
    }
    // 0 is AttachedStage's internal sentinel for "use the payload attach-time
    // ordinal". Storing it as the skip cursor leaves replay of the real attach
    // ordinal unguarded. The public contract is a caller-owned sealed ordinal.
    // Samples and the Python default start at 1.
    if (read_ordinal == 0) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "attach_ovstage: read_ordinal must be a caller-owned sealed ordinal; 0 is reserved");
    }

    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before attach_ovstage");
    }

    std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
    if (!instanceShared) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "attach_ovstage: invalid handle");
    }
    // Per-instance API contract: the caller must serialize calls on a single
    // handle (see AGENTS.md / the public header @note). This already-attached
    // check and the attach below are intentionally not locked against concurrent
    // foreground callers on the same instance.
    if (instanceShared->attachHandle != omni::physics::tensors::kNoAttach) {
        return set_error(OVPHYSX_API_ERROR,
                         "attach_ovstage: a stage is already attached; detach or reset before attaching ovstage");
    }

    omni::physx::IPhysxSimulation* physxSim =
        instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->attachOvstage) {
        return set_error(OVPHYSX_API_ERROR,
                         "attach_ovstage: IPhysxSimulation::attachOvstage is unavailable");
    }

    // A stage populated without the PhysX schemas carries none of the asset's Physx*
    // settings (self-collision, joint velocity limits, solver iterations, ...). Refuse
    // it here rather than simulate a scene the asset never authored. The setting
    // downgrades the refusal to a warning for a host that owns registration by a
    // route the probe cannot see.
    {
        std::string schemaError;
        const ovphysx_api_status_t schemaStatus = verify_physx_schemas_registered(schemaError);
        if (schemaStatus != OVPHYSX_API_SUCCESS) {
            bool requireRegistration = true;
            if (carb::Framework* fw = carb::getFramework()) {
                if (carb::settings::ISettings* settings = fw->tryAcquireInterface<carb::settings::ISettings>()) {
                    settings->setDefaultBool(kSettingRequireSchemaRegistration, true);
                    requireRegistration = settings->getAsBool(kSettingRequireSchemaRegistration);
                }
            }
            if (requireRegistration) {
                return set_error(schemaStatus, schemaError);
            }
            CARB_LOG_WARN("[ovphysx] %s (continuing: %s is false)", schemaError.c_str(),
                          kSettingRequireSchemaRegistration);
        }
    }

    instanceShared->ovstage_attach_payload.instance = stage;
    instanceShared->ovstage_attach_payload.dict = nullptr;
    instanceShared->ovstage_attach_payload.usdStageId = backing_ovstage_usd_stage_id_or_default(stage);

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
    bool attached = false;
    bool ownedByOther = false;
    {
        std::lock_guard<std::mutex> attachLock(g_gpuAttachMutex);
        if (g_liveAttachOwner != 0 && g_liveAttachOwner != handle) {
            ownedByOther = true;
        } else {
            applyAttachTimeGpuSelection(*instanceShared, carb::getFramework(), "attachOvstage");
            attached = physxSim->attachOvstage(&instanceShared->ovstage_attach_payload, read_ordinal);
            if (attached) {
                g_liveAttachOwner = handle;
                if (g_omniPvdRecordingOwner == OVPHYSX_INVALID_HANDLE &&
                    physxSim->isOmniPvdRecording && physxSim->isOmniPvdRecording())
                {
                    g_omniPvdRecordingOwner = handle;
                }
            }
        }
    }
    if (ownedByOther) {
        instanceShared->ovstage_attach_payload = OvstageAttachPayload{};
        return set_error(OVPHYSX_API_ERROR,
                         "attach_ovstage: another instance already owns the live PhysX attach; "
                         "detach it before attaching a new one");
    }
    if (!attached) {
        instanceShared->ovstage_attach_payload = OvstageAttachPayload{};
        return set_error(OVPHYSX_API_ERROR,
                         "attach_ovstage: IPhysxSimulation::attachOvstage failed");
    }

    // Two different things, deliberately read separately (ADR-0013): the backing
    // USD stage id, used only for USD-stage lifecycle below, and the attach handle,
    // which identifies this attach and is nonzero even when there is no stage.
    const int64_t stageId = physxSim->getAttachedStage
        ? static_cast<int64_t>(physxSim->getAttachedStage())
        : 0;
    const omni::physics::tensors::AttachHandle attachHandle =
        physxSim->getAttachHandle ? physxSim->getAttachHandle() : omni::physics::tensors::kNoAttach;

    instanceShared->attachedStageId = stageId;
    instanceShared->attachHandle = attachHandle;
    instanceShared->ovstage_attached = true;
    // Physics is attached whenever the attach succeeded. A zero stage id only
    // means there is no backing USD stage.
    instanceShared->resetStageFlags(attachHandle != omni::physics::tensors::kNoAttach);
    if (stageId != 0) {
        // Genuinely about the USD stage object, so it stays keyed by stage id.
        registerStageLifecycleEntry(stageId);
    }
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_update_from_ovstage(ovphysx_handle_t handle,
                                                        ovstage_ordinal_range_t range) {
    // ovstage's range: [start_ordinal, end_ordinal] when has_start_ordinal, else the
    // single end_ordinal. The runtime drain takes a closed [from, to].
    const uint64_t from_ordinal = range.has_start_ordinal ? range.start_ordinal : range.end_ordinal;
    const uint64_t to_ordinal = range.end_ordinal;
    if (from_ordinal > to_ordinal) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                         "update_from_ovstage: range.start_ordinal must be <= range.end_ordinal");
    }

    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before update_from_ovstage");
    }

    std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
    if (!instanceShared) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "update_from_ovstage: invalid handle");
    }
    if (!instanceShared->ovstage_attached) {
        return set_error(OVPHYSX_API_ERROR, "update_from_ovstage: no ovstage is attached");
    }

    omni::physx::IPhysxSimulation* physxSim =
        instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->updateFromOvStage) {
        return set_error(OVPHYSX_API_ERROR,
                         "update_from_ovstage: IPhysxSimulation::updateFromOvStage is unavailable");
    }

    const bool updated = physxSim->updateFromOvStage(from_ordinal, to_ordinal);
    if (!updated) {
        return set_error(OVPHYSX_API_ERROR,
                         "update_from_ovstage: IPhysxSimulation::updateFromOvStage failed");
    }

    return success();
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_clone(ovphysx_handle_t handle,
                                                   ovphysx_string_t source_path_in_usd,
                                                   ovphysx_string_t* target_paths,
                                                   uint32_t num_target_paths,
                                                   const float* anchor_transforms,
                                                   const uint32_t* env_ids) {
    OVPHYSX_NVTX_ZONE("ovphysx_clone");
    // Clone the source subtree to the target paths via the PhysX SDK replicator
    // (IPhysxSimulation::cloneEnvironments). Only path strings and a flat
    // [num_target_paths * 7] transform array cross the C ABI. anchor_transforms[i] is
    // the world pose of target_paths[i] (NULL co-locates on the source), and env_ids[i]
    // identifies copy i's logical environment (NULL means per-call numbering).

    // Failures here are synchronous (work runs inline), so report via set_enqueue_error
    // (status, no async op). Registering a failed op would orphan it: callers discard the
    // op_index on error, but wait_for_all_pending_ops() keeps failed ops, so it would fail
    // every later attach/reset/clone.
    if (!source_path_in_usd.ptr || source_path_in_usd.length == 0 || !target_paths || num_target_paths == 0) {
        return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                 "clone: source_path_in_usd must be non-empty, target_paths non-null, "
                                 "and num_target_paths > 0");
    }

    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_enqueue_error(wait_status, "Failed to complete pending operations before clone");
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
                                 "Call reset_stage(), then reload or reattach the source stage, "
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
    // Errors use set_enqueue_error (no async op) to avoid an orphaned pending op.
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

    // Firewall the extern "C" boundary: cloneEnvironments allocates, does USD ops, and runs
    // callbacks, any of which may throw. An exception crossing into ctypes/C callers can
    // terminate the process, so translate it into a failed API result.
    bool cloned = false;
    try {
        cloned = physxSim->cloneEnvironments(source_path.c_str(), targetPtrs.data(),
                                             num_target_paths, anchor_transforms, env_ids, useEnvIds);
    } catch (const std::exception& e) {
        return set_enqueue_error(OVPHYSX_API_ERROR, std::string("clone: cloneEnvironments threw: ") + e.what());
    } catch (...) {
        return set_enqueue_error(OVPHYSX_API_ERROR, "clone: cloneEnvironments threw a non-std exception");
    }
    if (!cloned) {
        return set_enqueue_error(OVPHYSX_API_ERROR,
                                 "clone: cloneEnvironments failed (the attach may have no backing USD stage, "
                                 "a target may already be populated with physics, or the source subtree is "
                                 "invalid) -- see the log for the specific cause");
    }

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

    // Record the now-live target paths so a later clone() on this attach rejects reusing them.
    instanceShared->cloned_target_paths.insert(targetStorage.begin(), targetStorage.end());

    // Register a completed op so wait_op(op_index) remains valid and succeeds immediately.
    async_event_handle_t event = AsyncEventManager::create_event();
    AsyncEventManager::complete_event(event, true);
    ovphysx_op_index_t op_index = ovphysx::async::register_operation(handle, event);
    return enqueue_success(op_index);
}

OVPHYSX_API ovphysx_result_t ovphysx_detach_ovstage(ovphysx_handle_t handle) {
    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before detach_ovstage");
    }

    std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
    if (!instanceShared) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "detach_ovstage: invalid handle");
    }
    const bool hadOvstage = instanceShared->ovstage_attached;

    if (hadOvstage) {
        const int64_t stageId = instanceShared->attachedStageId;
        const omni::physics::tensors::AttachHandle attachHandle = instanceShared->attachHandle;

        // ovx_primpath_t handles are owned by the attached Stage's path
        // dictionary. Drop the process-global visualization scope before the
        // runtime detaches (and the caller may destroy that dictionary).
        clearVisualizationScopeTokens();

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
            if (g_liveAttachOwner == handle) {
                if (physxSim && physxSim->detachStage) {
                    physxSim->detachStage();
                    reconcile_recording_owner_after_runtime_transition(physxSim);
                }
                g_liveAttachOwner = OVPHYSX_INVALID_HANDLE;
            } else {
                // Should be unreachable. attach_ovstage's owner latch means this
                // instance's attachHandle can only be live while it also holds
                // g_liveAttachOwner. detachStage() must not be called here because
                // it would tear down whichever other instance's attach is live.
                CARB_LOG_ERROR("[PHYSICS] detach_ovstage: instance %" PRIu64
                                " no longer owns the process-wide live attach; skipping runtime detach",
                                handle);
            }
        }

        if (stageId != 0) {
            markStageDetached(stageId, "detach_ovstage");
        }
        instanceShared->attachedStageId = 0;
        instanceShared->attachHandle = omni::physics::tensors::kNoAttach;
        instanceShared->ovstage_attached = false;
        instanceShared->ovstage_attach_payload = OvstageAttachPayload{};
        instanceShared->resetStageFlags();
    }

    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_get_attach_handle(ovphysx_handle_t instance_handle,
                                                      uint64_t* out_attach_handle) {
    if (!out_attach_handle)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "get_attach_handle: out_attach_handle is NULL");
    *out_attach_handle = omni::physics::tensors::kNoAttach;

    // Surfaces the handle recorded at attach. No pending-op wait, because this
    // reads attach identity, not simulation state, and the value is written only
    // by ovphysx_attach_ovstage() and cleared by ovphysx_detach_ovstage().
    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
    InstanceData* instance = get_instance_ptr(instance_handle);
    if (!instance)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "get_attach_handle: invalid handle");

    *out_attach_handle = static_cast<uint64_t>(instance->attachHandle);
    return success();
}

OVPHYSX_API ovphysx_result_t ovphysx_find_prims(ovphysx_handle_t handle,
                                                        ovphysx_string_t path_pattern,
                                                        ovphysx_string_t attribute_name,
                                                        ovphysx_prim_list_t* out_prim_list) {
    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_error(wait_status, "Failed to complete pending operations before find_prims");
    }

    return set_error(OVPHYSX_API_NOT_IMPLEMENTED, "find_prims not yet implemented");
}

OVPHYSX_API ovphysx_result_t ovphysx_destroy_prim_list(ovphysx_handle_t handle,
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

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_add_user_task(ovphysx_handle_t handle,
                                                                   const ovphysx_user_task_desc_t* desc) {
    if (!desc || !desc->run) {
        return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid task description");
    }

    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_enqueue_error(wait_status, "Failed to complete pending operations before add_user_task");
    }
    
    async_event_handle_t event = AsyncEventManager::create_event();
    
    // Execute task immediately (could be queued in future)
    ovphysx_op_index_t op_index = ovphysx::async::register_operation(handle, event);
    ovphysx_result_t result = desc->run(handle, op_index, desc->user_data);
    
    const bool task_succeeded = (result.status == OVPHYSX_API_SUCCESS);
    std::string event_error;
    if (!task_succeeded) {
        // The user task stored its error in TLS via set_error().
        auto& last_err = tls_error().last_error;
        if (!last_err.empty()) {
            event_error = last_err;
        }
    }
    AsyncEventManager::complete_event(event, task_succeeded, event_error.empty() ? nullptr : event_error.c_str());

    if (!task_succeeded) {
        tls_error().last_error = event_error.empty() ? "User task failed" : event_error;
        return {result.status, op_index};
    }
    return enqueue_success(op_index);
}

OVPHYSX_API ovphysx_result_t ovphysx_wait_op(ovphysx_handle_t handle,
                                                     ovphysx_op_index_t op_index,
                                                     ovphysx_timeout_t timeout_ns,
                                                     ovphysx_op_wait_result_t* out_wait_result) {
    OVPHYSX_NVTX_ZONE("ovphysx_wait_op");
    // Clear per-op errors at the start of each wait_op call
    tls_error().op_errors.clear();

    // Infinite waits in the RL hot loop (step -> wait_op -> reads/writes -> step)
    // can skip the generic get_pending_ops/event machinery when the requested
    // simulation is the only tracked op. Poll and finite waits stay on the
    // generic path so checkResults() can enforce their readiness budget.
    if (op_index != OVPHYSX_OP_INDEX_ALL && timeout_ns == OVPHYSX_TIMEOUT_INFINITE) {
        std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
        async_event_handle_t simulation_event = 0;
        if (instanceShared) {
            std::lock_guard<std::mutex> op_lock(instanceShared->op_tracking_mutex);
            if (instanceShared->op_to_event.size() == 1) {
                std::unordered_map<ovphysx_op_index_t, async_event_handle_t>::iterator it =
                    instanceShared->op_to_event.find(op_index);
                if (it != instanceShared->op_to_event.end()) {
                    std::lock_guard<std::mutex> simulation_lock(instanceShared->simulationMutex);
                    if (instanceShared->pendingSimulationEvent != 0 &&
                        it->second == instanceShared->pendingSimulationEvent) {
                        // Claim the single-use index before blocking so a competing
                        // waiter observes NOT_FOUND instead of consuming it again.
                        simulation_event = it->second;
                        if (op_index > instanceShared->last_explicitly_consumed_op_index)
                            instanceShared->last_explicitly_consumed_op_index = op_index;
                        instanceShared->op_to_event.erase(it);
                    }
                }
            }
        }
        if (simulation_event != 0) {
            // This is the simulation event, so sync directly.
            ovphysx_api_status_t sync_status = omni_sdk_physx_sync(handle);
            AsyncEventManager::cleanup_event(simulation_event);
            instanceShared->all_ops_synced.store(true, std::memory_order_release);
            if (out_wait_result) {
                out_wait_result->error_op_indices = nullptr;
                out_wait_result->num_errors = 0;
                out_wait_result->lowest_pending_op_index = 0;
            }
            if (sync_status != OVPHYSX_API_SUCCESS) {
                tls_error().op_errors[op_index] = "Simulation sync failed";
                if (out_wait_result) {
                    try {
                        out_wait_result->error_op_indices = new ovphysx_op_index_t[1]{op_index};
                        out_wait_result->num_errors = 1;
                    } catch (const std::bad_alloc&) {
                        return set_error(OVPHYSX_API_ERROR, "Out of memory allocating wait_op error indices");
                    }
                }
                return set_error(sync_status, "Simulation sync failed");
            }
            return success();
        }
    }

    // Generic path for non-simulation events or OVPHYSX_OP_INDEX_ALL
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        if (g_instances.find(handle) == g_instances.end()) {
            if (out_wait_result) {
                out_wait_result->error_op_indices = nullptr;
                out_wait_result->num_errors = 0;
                out_wait_result->lowest_pending_op_index = 0;
            }
            return set_error(OVPHYSX_API_NOT_FOUND, "Invalid handle");
        }
    }

    if (op_index == OVPHYSX_OP_INDEX_ALL) {
        // ALL explicitly acknowledges any prefix already completed by internal
        // stream synchronization, even when no tracked operations remain.
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (instance) {
            std::lock_guard<std::mutex> lock(instance->op_tracking_mutex);
            if (instance->last_internally_synced_op_index >
                instance->last_explicitly_consumed_op_index) {
                instance->last_explicitly_consumed_op_index =
                    instance->last_internally_synced_op_index;
            }
        }
    } else {
        bool found = false;
        bool internally_synced = false;
        {
            std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
            InstanceData* instance = get_instance_ptr(handle);
            if (instance) {
                std::lock_guard<std::mutex> lock(instance->op_tracking_mutex);
                found = (instance->op_to_event.find(op_index) != instance->op_to_event.end());
                if (found && op_index > 0) {
                    const ovphysx_op_index_t internally_synced_prefix =
                        std::min(instance->last_internally_synced_op_index, op_index - 1);
                    if (internally_synced_prefix > instance->last_explicitly_consumed_op_index)
                        instance->last_explicitly_consumed_op_index = internally_synced_prefix;
                }
                if (!found && op_index != 0 &&
                    op_index > instance->last_explicitly_consumed_op_index &&
                    op_index <= instance->last_internally_synced_op_index) {
                    internally_synced = true;
                    instance->last_explicitly_consumed_op_index = op_index;
                }
            }
        }
        if (!found && internally_synced) {
            if (out_wait_result) {
                out_wait_result->error_op_indices = nullptr;
                out_wait_result->num_errors = 0;
                out_wait_result->lowest_pending_op_index = 0;
            }
            return success();
        }
        if (!found) {
            if (out_wait_result) {
                out_wait_result->error_op_indices = nullptr;
                out_wait_result->num_errors = 0;
                out_wait_result->lowest_pending_op_index = 0;
            }
            return set_error(OVPHYSX_API_NOT_FOUND, "op_index not found (already consumed or never existed)");
        }
    }

    std::vector<ovphysx_op_index_t> pending_ops = ovphysx::async::get_pending_ops(handle, op_index);

    if (pending_ops.empty()) {
        if (out_wait_result) {
            out_wait_result->error_op_indices = nullptr;
            out_wait_result->num_errors = 0;
            out_wait_result->lowest_pending_op_index = 0;
        }
        if (op_index == OVPHYSX_OP_INDEX_ALL)
            return success();
        return set_error(OVPHYSX_API_NOT_FOUND, "op_index not found");
    }

    const bool wait_forever = (timeout_ns == OVPHYSX_TIMEOUT_INFINITE);
    const std::chrono::nanoseconds timeout_duration = clamp_timeout_ns(timeout_ns);
    const std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    // Failed op indices. Error messages go into the TLS op_errors map.
    std::vector<ovphysx_op_index_t> collected_error_indices;

    ovphysx_op_index_t lowest_pending = 0;
    for (ovphysx_op_index_t pending_op : pending_ops) {
        // Calculate remaining timeout (allow zero to still poll once)
        ovphysx_timeout_t remaining_timeout_ns = OVPHYSX_TIMEOUT_POLL;
        const std::chrono::steady_clock::duration elapsed = std::chrono::steady_clock::now() - start_time;
        if (wait_forever) {
            remaining_timeout_ns = OVPHYSX_TIMEOUT_INFINITE;
        } else if (timeout_duration.count() == 0 || elapsed >= timeout_duration) {
            remaining_timeout_ns = OVPHYSX_TIMEOUT_POLL;
        } else {
            remaining_timeout_ns = static_cast<ovphysx_timeout_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout_duration - elapsed).count());
        }

        async_event_handle_t event = ovphysx::async::get_event_for_op(handle, pending_op);
        std::string error_msg;
        ovphysx_api_status_t wait_status = wait_on_single_event(handle, pending_op, event, remaining_timeout_ns, error_msg, /*consume_op_index=*/true);

        if (wait_status == OVPHYSX_API_TIMEOUT) {
            lowest_pending = pending_op;
            break;
        } else if (wait_status == OVPHYSX_API_ERROR) {
            tls_error().op_errors[pending_op] = error_msg.empty() ? "Operation failed" : error_msg;
            collected_error_indices.push_back(pending_op);
            // Continue waiting on remaining operations to collect all errors
        }
        // OVPHYSX_API_SUCCESS - continue to next operation
    }

    if (out_wait_result) {
        if (collected_error_indices.empty()) {
            out_wait_result->error_op_indices = nullptr;
            out_wait_result->num_errors = 0;
        } else {
            // Allocate error index array (user must call ovphysx_destroy_wait_result to free).
            ovphysx_op_index_t* error_indices = nullptr;
            try
            {
                error_indices = new ovphysx_op_index_t[collected_error_indices.size()];
            }
            catch (const std::bad_alloc&)
            {
                out_wait_result->error_op_indices = nullptr;
                out_wait_result->num_errors = 0;
                out_wait_result->lowest_pending_op_index = lowest_pending;
                return set_error(OVPHYSX_API_ERROR, "Out of memory allocating wait_op error indices");
            }

            for (size_t i = 0; i < collected_error_indices.size(); ++i)
            {
                error_indices[i] = collected_error_indices[i];
            }

            out_wait_result->error_op_indices = error_indices;
            out_wait_result->num_errors = collected_error_indices.size();
        }
        out_wait_result->lowest_pending_op_index = lowest_pending;
    }

    if (lowest_pending != 0) {
        tls_error().last_error.clear();
        return {OVPHYSX_API_TIMEOUT};
    }

    if (!collected_error_indices.empty()) {
        return set_error(OVPHYSX_API_ERROR, "One or more operations failed");
    }

    std::shared_ptr<InstanceData> instance_shared = get_instance(handle);
    if (instance_shared) {
        std::lock_guard<std::mutex> op_lock(instance_shared->op_tracking_mutex);
        if (instance_shared->op_to_event.empty())
            instance_shared->all_ops_synced.store(true, std::memory_order_release);
    }

    return success();
}

OVPHYSX_API ovphysx_string_t ovphysx_get_last_error(void) {
    auto& err = tls_error().last_error;
    if (err.empty()) return {"", 0};
    return {err.c_str(), err.size()};
}

OVPHYSX_API ovphysx_string_t ovphysx_get_last_op_error(ovphysx_op_index_t op_index) {
    auto& op_errors = tls_error().op_errors;
    auto it = op_errors.find(op_index);
    if (it == op_errors.end() || it->second.empty()) return {"", 0};
    return {it->second.c_str(), it->second.size()};
}

OVPHYSX_API void ovphysx_destroy_wait_result(ovphysx_op_wait_result_t* result) {
    if (result) {
        delete[] result->error_op_indices;
        result->error_op_indices = nullptr;
        result->num_errors = 0;
    }
}

} // extern "C"

// Log capture API and logging configuration are implemented in LogManager.cpp
