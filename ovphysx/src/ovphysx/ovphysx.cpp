// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "internal/CpuFeatureCheck.h"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"

#include "CarboniteLoader/CarboniteLoader.hpp"
#include "cuda_shim/CudaShim.h"
#include <omni/physx/PhysXRuntime.h>
#include "UsdSchemaPaths/UsdSchemaPaths.h"
#include "UsdVersionCheck/UsdVersionCheck.h"
#include "AsyncEventManager/AsyncEventManager.hpp"
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
#include "internal/sdk/LibraryPathUtils.hpp"

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
#include <pxr/usd/sdf/path.h>

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
// Private C API declarations for internal tensor loader (C header)
#include "internal/sdk/ovphysxSDK.h"
// Internal sidecar API + loader (shared between main library and replicator)
#include "internal/sdk/ovphysxSDKSidecarLoader.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"
// Global instances map definition - stores shared_ptr for safe lifetime management
std::unordered_map<ovphysx_handle_t, std::shared_ptr<InstanceData>> g_instances;

// Global reader-writer lock to protect g_instances map access
// - Use shared_lock (read) for lookups (allows concurrent reads)
// - Use unique_lock (write) for insert/erase (exclusive access)
std::shared_mutex g_instances_mutex;

namespace
{

// The one process-wide serial sequence behind every ovphysx-owned opaque object
// handle: instances, tensor bindings, contact bindings and SDF views.
//
// It is process-wide on purpose. NVBug 6504951 is an ACROSS-OWNER aliasing bug:
// the previous implementation used independent counters that shared the same
// numeric space, so values from different object kinds and different instances
// could collide. An instance handle and that instance's first tensor binding
// were both the number 1, and a binding handle from a destroyed instance
// matched the first binding of the next instance.
//
// One never-reused sequence makes the four handle kinds numerically unique for
// the life of the process. Its process-wide scope prevents stale tokens and
// tokens from another kind or owner from aliasing a different ovphysx-owned
// object. Handles stay opaque uint64_t and 0 stays the invalid sentinel.
//
// Deliberately a single internally-linked constant-initialized atomic: no
// mutex, no pointer, no singleton, and therefore no static-initialization or
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

// Serializes the /physics/cudaDevice write and physxSim->attachStage() so concurrent
// instances with different active_cuda_gpus ordinals don't stomp each other's setting.
std::mutex g_gpuAttachMutex;

// Process-wide CPU-only mode. Set via ovphysx_set_cpu_mode(true) before any instances
// are created. When true, IPhysxFoundation::setCpuMode(true) is applied at first attach,
// preventing any CUDA driver contact for the process lifetime.
static std::atomic<bool> g_forceCpuMode{false};

// True when this process must never touch the GPU: either CPU-only mode was forced via
// ovphysx_set_cpu_mode(true) or the OVPHYSX_DISABLE_GPU env var is set. Both inputs are
// fixed for the process lifetime once instances exist, so callers on hot paths may cache
// the result. Whether a usable CUDA device is actually present is a separate check
// through the CPU-only-safe CUDA shim.
// Declared in ovphysxSDK.hpp so other translation units (e.g. ovphysxTensorBinding.cpp)
// gate GPU-only work on the same predicate rather than re-checking the env var alone.
bool isProcessGpuDisabled()
{
    // The env var is fixed for the process lifetime, so read it once. g_forceCpuMode
    // can still be toggled by ovphysx_set_cpu_mode() (before instances exist), so it is
    // loaded live. This keeps the predicate a single atomic load on per-step call sites.
    static const bool envDisabled = std::getenv("OVPHYSX_DISABLE_GPU") != nullptr;
    return g_forceCpuMode.load(std::memory_order_acquire) || envDisabled;
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
// Empty input returns {0} (default: GPU 0).
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
        // skip leading whitespace
        while (p < end && (*p == ' ' || *p == '\t')) ++p;
        if (p >= end) break;

        // optional leading minus
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

        // skip trailing whitespace before comma
        while (p < end && (*p == ' ' || *p == '\t')) ++p;
        if (p < end)
        {
            if (*p == ',')
            {
                ++p;
                // skip whitespace after comma, then check for trailing comma
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

    // reject duplicates
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

// Lazily call attachStage() if not yet done for the
// current stage.  Also performs the initial PhysX scene parse (simulate(0,0) +
// fetchResults) so that TensorAPI can discover prims.  Attachment is deferred
// until the caller has drained ovstage edits.
//
// Called from simulate(), warmup_gpu(), and create_tensor_binding().
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
        // All attaches hold g_gpuAttachMutex so concurrent instances with different
        // active_cuda_gpus ordinals don't stomp each other's /physics/cudaDevice setting.
        std::unique_lock<std::mutex> attachLock(g_gpuAttachMutex);

        // Re-check under the lock: a concurrent ensure_physics_attached() for this same
        // handle may have completed the attach while we waited on attachLock. Without this
        // re-check we would write the process-global settings and call attachStage() a
        // second time on an already-attached stage.
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

            // Write /physics/cudaDevice if the caller restricted GPU ordinals.
            // Skipped when GPU is disabled for the process (ovphysx_set_cpu_mode(true)
            // or OVPHYSX_DISABLE_GPU): active_cuda_gpus is meaningless in CPU-only mode,
            // and the multi-GPU branch below probes the driver via the CUDA shim, which
            // would violate the "no CUDA driver touch" contract. Mirrors the same gate on
            // the create-time ordinal validation in createInstanceInternal().
            // Ordinals were parsed and validated at create time (active_cuda_ordinals);
            // empty means the caller did not restrict GPU ordinals.
            const std::vector<int32_t>& ords = instance->active_cuda_ordinals;
            if (!ords.empty() && !isProcessGpuDisabled())
            {
                if (auto* settings = framework->tryAcquireInterface<carb::settings::ISettings>())
                {
                    const int32_t gpuIndex = ords[0];
                    settings->setInt("/physics/cudaDevice", gpuIndex);
                    CARB_LOG_INFO("[ovphysx] /physics/cudaDevice=%d before attachStage (stage=%" PRId64 ")", gpuIndex, stageId);

                    if (ords.size() > 1)
                    {
                        int count = 0;
                        const CUresult deviceCountResult = omni::physx::cudaShim::cuDeviceGetCount_(&count);
                        if (deviceCountResult == CUDA_SUCCESS && count > 0)
                        {
                            char modeErr[384] = {};
                            int32_t multiGPUMode = determineMultiGPUMode(ords, count, modeErr, sizeof(modeErr));
                            if (multiGPUMode >= 0)
                                settings->setInt("/physics/sceneMultiGPUMode", multiGPUMode);
                            else
                                // Unsupported multi-GPU pattern (e.g. non-contiguous ordinals).
                                // The first ordinal still drives /physics/cudaDevice above, so we
                                // proceed single-GPU rather than fail the attach -- but surface the
                                // misconfiguration instead of silently dropping modeErr.
                                CARB_LOG_WARN("[ovphysx] active_cuda_gpus multi-GPU mode not applied: %s", modeErr);
                        }
                    }
                }
            }

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
            CARB_LOG_INFO("[PHYSICS] Lazy attachStage() completed for stage %" PRId64, stageId);
        }
    }

    // Initial scene parse -- PhysX needs a simulate()+fetchResults() cycle
    // to discover articulations, joints, etc. from the attached scene.
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

// Instance-aware simulation function
// This initiates simulation but does NOT wait for results (returns quickly)
// Non-static: also used by ovphysxTensorBinding.cpp for auto-warmup
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

        // PhysX simulate() can be called multiple times safely, but we can only track
        // one pending event, so complete/cleanup any previous one first.
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

            // Store pending state (don't call fetchResults here!)
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

    // Sync function - waits for pending simulation to complete
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

        const bool hasPhysicsStage = (instanceShared->attachedStageId != 0) || instanceShared->ovstage_attached;

        // WARNING: Calling fetchResults() with no USD stage loaded causes PhysX to hang!
        // Guard against it by checking for an attached stage first; with no stage this is a no-op, not an error.
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
    // Typed config system: enum → Carbonite path lookup tables
    // ========================================================================
    static const char* s_boolKeyPaths[] = {
        "/physics/disableContactProcessing",
        "/physics/collisionConeCustomGeometry",
        "/physics/collisionCylinderCustomGeometry",
        "/physics/omniPvdOutputEnabled",
    };
    static_assert(std::size(s_boolKeyPaths) == OVPHYSX_CONFIG_BOOL_COUNT, "s_boolKeyPaths out of sync with enum");

    static const char* s_int32KeyPaths[] = {
        "/physics/numThreads",
        "/physics/sceneMultiGPUMode",
    };
    static_assert(std::size(s_int32KeyPaths) == OVPHYSX_CONFIG_INT32_COUNT, "s_int32KeyPaths out of sync with enum");

    static const char* s_floatKeyPaths[] = {
        nullptr,
    };
    static_assert(OVPHYSX_CONFIG_FLOAT_COUNT == 0, "Update s_floatKeyPaths when float keys are added");

    static const char* s_stringKeyPaths[] = {
        "/persistent/physics/omniPvdOvdRecordingDirectory",
        "/UJITSO/datastore/localCachePath",
    };
    static_assert(std::size(s_stringKeyPaths) == OVPHYSX_CONFIG_STRING_COUNT, "s_stringKeyPaths out of sync with enum");

    // Forward declaration (defined below).
    static void applySettingValue(carb::settings::ISettings* settings, const char* key, const char* value);

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
                std::string val(entry.value.string_value.ptr, entry.value.string_value.length);
                settings->setString(s_stringKeyPaths[entry.key.string_key], val.c_str());
                CARB_LOG_INFO("[Config] Set string %s = %s", s_stringKeyPaths[entry.key.string_key], val.c_str());
            }
            return OVPHYSX_API_SUCCESS;
        case OVPHYSX_CONFIG_KEY_TYPE_CARBONITE:
            if (!entry.key.carbonite_key.ptr || !entry.value.string_value.ptr)
                return OVPHYSX_API_INVALID_ARGUMENT;
            {
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
                applySettingValue(settings, key.c_str(), val.c_str());
            }
            return OVPHYSX_API_SUCCESS;
        default:
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
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

    // Helper to detect type and apply a setting value
    // Supports: bool ("true"/"false"), int, float, string
    static void applySettingValue(carb::settings::ISettings* settings, const char* key, const char* value)
    {
        if (!settings || !key || !value) return;
        
        // Try bool first ("true" or "false")
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
            // Only treat overflow as terminal when the entire string was a
            // pure integer (ptr reached the end).  If ptr stopped at e.g. '.'
            // the value may be a float like "99999999999.5" -- fall through.
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
        
        // Default: treat as string
        settings->setString(key, value);
        CARB_LOG_INFO("[Settings] Set string %s = %s", key, value);
    }
    
    // Helper to get a setting value as string
    // Returns true if setting exists, false otherwise
    static bool getSettingValueAsString(carb::settings::ISettings* settings, const char* key,
                                        char* value_out, uint32_t value_out_size,
                                        size_t* out_required_size = nullptr)
    {
        if (!settings || !key || !value_out || value_out_size == 0) return false;

        // Helper: write the formatted value into the caller's buffer and
        // optionally report the full size (including null terminator) via
        // out_required_size.  Returns true (setting exists).
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
            // Already handled above, but just in case
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
    // Clamp uint64 timeout values to std::chrono::nanoseconds range.
    static std::chrono::nanoseconds clamp_timeout_ns(uint64_t timeout_ns) {
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
                // RTLD_NOLOAD still bumps the refcount on a match; balance it so repeated
                // symbol lookups don't leak references and pin libovstage past unload. The
                // library stays mapped via its real owner, so proc remains valid.
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
    
    // Wait on a single event with timeout, handling simulation completion if needed
    // Returns: OVPHYSX_API_SUCCESS, OVPHYSX_API_TIMEOUT, or OVPHYSX_API_ERROR
    // If the operation failed, error_out will be populated with the error message
    static ovphysx_api_status_t wait_on_single_event(ovphysx_handle_t handle, 
                                                      ovphysx_op_index_t op_index,
                                                      async_event_handle_t event,
                                                      uint64_t timeout_ns,
                                                      std::string& error_out,
                                                      bool consume_op_index) {
        if (event == 0) {
            return OVPHYSX_API_ERROR;
        }
        
        // Calculate timeout parameters upfront for proper poll semantics
        auto timeout = clamp_timeout_ns(timeout_ns);
        const bool no_wait = (timeout.count() == 0);
        
        bool needsFetchResults = false;
        {
            std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
            InstanceData* instance = get_instance_ptr(handle);
            if (instance) {
                needsFetchResults = (instance->pendingSimulationEvent == event);
            }
        }

        // IMPORTANT: For non-blocking poll (timeout=0), check completion status first
        // and only call fetchResults if we're willing to block.
        if (needsFetchResults) {
            if (no_wait) {
                async_status_t status = async_poll_event(event);
                if (status == ASYNC_STATUS_PENDING) {
                    return OVPHYSX_API_TIMEOUT;
                }
                // If already completed/failed, fall through to handle result
            }

            std::shared_ptr<InstanceData> instanceShared = get_instance(handle);
            if (instanceShared) {
                std::unique_lock<std::mutex> instance_lock(instanceShared->simulationMutex);

                auto physxSim = instanceShared->carbonite->getPhysxSimulation();

                if (physxSim && (instanceShared->attachedStageId != 0 || instanceShared->ovstage_attached)) {
                    try {
                        // Blocks until simulation completes (or returns quickly if already done)
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
        auto start = std::chrono::steady_clock::now();
        
        while (true) {
            async_status_t status = async_poll_event(event);
            if (status == ASYNC_STATUS_COMPLETED || status == ASYNC_STATUS_FAILED) {
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

            auto elapsed = std::chrono::steady_clock::now() - start;
            if (elapsed >= timeout) {
                return OVPHYSX_API_TIMEOUT;
            }
            // Brief sleep to avoid busy-waiting
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
    
    // Wait for all pending operations to complete (for stream-ordered execution)
    // Returns OVPHYSX_API_SUCCESS if all operations completed, or error status.
    // On success, all waited ops are consumed from the tracking map and their events
    // are cleaned up, preventing unbounded growth in long-running loops. A high-water
    // mark lets a later wait_op() observe an operation consumed by internal synchronization.
    static ovphysx_api_status_t wait_for_all_pending_ops(ovphysx_handle_t handle) {
        std::vector<ovphysx_op_index_t> pending_ops = ovphysx::async::get_pending_ops(handle, OVPHYSX_OP_INDEX_ALL);

        if (pending_ops.empty()) {
            return OVPHYSX_API_SUCCESS;
        }

        // Wait on each operation without timeout (don't consume yet - wait for all first)
        for (ovphysx_op_index_t pending_op : pending_ops) {
            // Event may be 0 for CUDA-only ops (tensor binding async). wait_on_single_event
            // handles CUDA events even when this is not an AsyncEventManager event.
            async_event_handle_t event = ovphysx::async::get_event_for_op(handle, pending_op);

            std::string error_msg;
            ovphysx_api_status_t status = wait_on_single_event(handle, pending_op, event, UINT64_MAX, error_msg, /*consume_op_index=*/false);
            if (status != OVPHYSX_API_SUCCESS) {
                // Operation failed - this is an error in stream-ordered execution.
                // Leave failed and subsequent ops in the map so the user can retrieve
                // errors via wait_op().
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
}

// Internal C++ function implementation (exposed via ovphysxSDK.hpp)
ovphysx_api_status_t omni_sdk_physx_wait_all_pending_internal(ovphysx_handle_t handle) {
    // Fast path: if all_ops_synced is already true, we know nothing is pending.
    // Avoids the 2-mutex + vector-alloc overhead of wait_for_all_pending_ops.
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

            // Detach PhysX simulation.
            // When physics was never explicitly attached (deferred path), we must
            // still attach+detach here: scene population during load_usd creates
            // state that PhysX observes internally, and only detachStage clears it.
            // Skipping detach leaves dangling refs that corrupt later simulate().
            {
                omni::physx::IPhysxSimulation* physxSim =
                    instance->carbonite ? instance->carbonite->getPhysxSimulation() : nullptr;

                if (!instance->physics_attached.load(std::memory_order_acquire)) {
                    // Pair a quick attach+detach so the PhysX plugin releases
                    // any refs it accumulated during load.
                    if (physxSim)
                        physxSim->attachStage(stageId);
                    CARB_LOG_INFO("[UNLOAD #%u] Stage %" PRId64 " - late attach for clean detach (physics was deferred)", unloadSeq, stageId);
                }

                if (physxSim) {
                    CARB_LOG_INFO("[UNLOAD #%u] Detaching PhysX simulation for stage %" PRId64, unloadSeq, stageId);
                    physxSim->detachStage();
                    CARB_LOG_INFO("[UNLOAD #%u] PhysX simulation detached for stage %" PRId64, unloadSeq, stageId);
                } else {
                    CARB_LOG_WARN("[UNLOAD #%u] WARNING: PhysX simulation interface unavailable for stage %" PRId64, unloadSeq, stageId);
                }
            }
            
            
            // Release the tensor SimulationBackend's per-stage data before clearing the stage id.
            // Retained from the legacy unload path; keeps the tensor backend from holding stale
            // views across reset / reattach.
            if (omni::physics::tensors::TensorApi* tensorApi =
                    omni::physx::runtime::tryGetTensorApiInterface()) {
                if (tensorApi->resetStage)
                    tensorApi->resetStage(stageId);
            }

            instance->attachedStageId = 0;
            instance->resetStageFlags();
            ovphysx_close_usd_stage_wrapper(stageId);
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
    // Wait for all pending operations, then unload (these acquire their own locks)
    wait_for_all_pending_ops(handle);  // Ignore return value - continue cleanup even if wait fails

    bool detachOvstage = false;
    // Capture the attached stageId before detach/unload clear it. This is used
    // as a TensorAPI cleanup fallback when unload exits before clearing the stage.
    int64_t destroyedStageId = 0;
    {
        std::shared_lock<std::shared_mutex> mapLock(g_instances_mutex);
        auto it = g_instances.find(handle);
        if (it != g_instances.end() && it->second)
        {
            destroyedStageId = it->second->attachedStageId;
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
        }
    }

    {
        ScopedUnloadCaller callerScope(UnloadCaller::kDestroy);
        omni_sdk_physx_unload_usd(handle);  // Ignore return value - continue cleanup even if unload fails
    }

    bool stageUnloadCompleted = false;
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        auto it = g_instances.find(handle);
        stageUnloadCompleted = it != g_instances.end() && it->second && it->second->attachedStageId == 0;
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
        // Per-instance fallback: release sim data if unload did not clear this stage.
        // Unlike the full reset() below, this needs no g_createInstanceMutex
        // serialization: it only touches destroyedStageId's entries, and this stage
        // was already detached above, so a concurrent create_instance+stage ingest (which
        // populates a *different* stageId) cannot collide.
        // SimulationBackend::resetStage takes its own per-backend lock.
        if (tensorApi->resetStage && destroyedStageId > 0 && !stageUnloadCompleted)
            tensorApi->resetStage(destroyedStageId);

        // On last instance: full backend reset to release any remaining resources.
        // reset() clears sim data for ALL stages, so it must not race a concurrent
        // create_instance+ovstage ingest that has populated a new stage's data. Hold
        // g_createInstanceMutex across the emptiness re-check AND the reset() call:
        // create_instance holds the same mutex while inserting into g_instances, so
        // this serialization guarantees no instance can appear between the re-check
        // and reset(). Lock order (create mutex then instances mutex) matches
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
        CARB_LOG_INFO("[ovphysx] Static runtime kept alive until process exit");
    }

    return OVPHYSX_API_SUCCESS;
}

// ==================== NEW API FUNCTIONS ====================

OVPHYSX_API ovphysx_result_t ovphysx_register_schema_paths(void)
{
    std::string error;
    if (!omni::sdk::usd_schema_paths::registerSchemaPathsOnce(&error))
    {
        return set_error(OVPHYSX_API_ERROR, error);
    }
    return success();
}

// Main SDK bootstrap entry point.
// It serializes process-global setup, starts
// Carbonite, reuses or preloads namespaced USD, applies user config, then loads
// ovphysx's own PhysX plugins. It rejects a process that already loaded a PhysX
// Carbonite stack because only namespaced USD is shared with other OV libraries.
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

    // Create instance data as shared_ptr for safe lifetime management
    auto instanceData = std::make_shared<InstanceData>();


    // Parse active_cuda_gpus early to fail fast on bad syntax before bringing up
    // Carbonite/PhysX. The parsed ordinals are range-validated post-init (device
    // count) and stored on the instance for the deferred /physics/cudaDevice write
    // at first attach. CarboniteLoader is given the fixed -2 sentinel below, not an
    // ordinal from this list.
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
        // parseActiveCudaGpus returns {0} for empty input and a non-empty vector
        // on success; an empty result only occurs on error (errbuf filled),
        // which is handled above. Guard here defensively.
        if (requestedOrdinals.empty())
        {
            return set_error(OVPHYSX_API_INVALID_ARGUMENT,
                             "active_cuda_gpus: internal parse error (empty ordinal list)");
        }
    }

    // ========================================================================
    // Configure startup settings that MUST be applied before PhysX plugins load.
    //
    // Note: carb.settings.plugin is loaded by CarboniteLoader, so we cannot set these
    // directly via ISettings yet. Instead, we pass them to CarboniteLoader which applies
    // them immediately after carb.settings.plugin loads, but BEFORE PhysX plugins load.
    // ========================================================================
    {
        // Pass -2 to CarboniteLoader so it skips loading omni.gpucompute-cuda.plugin
        // and related Carbonite GPU plugins at startup. PhysX manages its own GPU
        // context via the PhysX SDK directly and does not depend on those plugins.
        // /physics/cudaDevice (the GPU ordinal) is written separately below, after
        // plugin load, inside g_createInstanceMutex.
        int32_t cudaDevice = -2;

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

        ovphysx::CarboniteLoader::setStartupCudaDevice(cudaDevice);
        // Note: /physics/suppressReadback (DirectGPU-API mode) is opt-in by
        // the host; ovphysx never writes it. See create_args doc-comment in
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

    // Load the SDK config after Carbonite startup. In this namespaced-only
    // branch this does not validate or attach to a host/classic USD runtime.
    // This loads config.toml and makes it available to later setup code.
    ovphysx_result_t usd_init_result = omni::sdk::usd_version::initializeUsdVersionCheck();
    if (usd_init_result.status != OVPHYSX_API_SUCCESS) {
        // initializeUsdVersionCheck() already set a detailed TLS error; propagate it as-is.
        return usd_init_result;
    }

    // Reuse an already-loaded OV namespaced USD monolith when one exists,
    // otherwise preload the packaged SDK USD libs now.
    CARB_LOG_INFO("[USD Compatibility] Ensuring SDK USD libs are preloaded");
    if (!instanceData->carbonite->preloadUsdLibraries()) {
        CARB_LOG_ERROR("Failed to preload USD for ovphysx");
        return set_error(OVPHYSX_API_ERROR, "Failed to preload USD libraries");
    }
    if (!instanceData->carbonite->loadUsdDependentPlugins()) {
        CARB_LOG_WARN("[ovphysx] Failed to load USD-dependent plugins; USD-dependent operations may fail");
    }

    // Apply user config entries BEFORE loading PhysX plugins. OmniPVD recording
    // is initialized during createPhysics() (triggered by loadPhysxPlugins), so
    // settings like omniPvdOutputEnabled must already be in place.
    if (args && args->config_entry_count > 0 && args->config_entries) {
        auto* framework = carb::getFramework();
        auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
        if (settings) {
            CARB_LOG_INFO("[ovphysx] Applying %u user config entries (pre-plugin-load)", args->config_entry_count);
            for (uint32_t i = 0; i < args->config_entry_count; ++i) {
                ovphysx_api_status_t status = applyConfigEntry(settings, args->config_entries[i]);
                if (status != OVPHYSX_API_SUCCESS) {
                    CARB_LOG_WARN("[ovphysx] Failed to apply config entry %u (status=%d)", i, status);
                }
            }
        } else {
            CARB_LOG_WARN("[ovphysx] Warning: ISettings not available, cannot apply user config entries");
        }
    }

    if (!instanceData->carbonite->loadPhysxPlugins()) {
        const std::string& loaderError = instanceData->carbonite->getLastError();
        const char* message = loaderError.empty() ? "Failed to load PhysX plugins after USD preload"
                                                  : loaderError.c_str();
        CARB_LOG_ERROR("%s", message);
        return set_error(OVPHYSX_API_ERROR, message);
    }

    // Preload the internal sidecar so its carb::Framework + OmniCore built-ins are seeded
    // before first use. CarboniteLoader has already made the USD monolith globally visible,
    // so the sidecar binds to that runtime without pulling in another USD image.
    CARB_LOG_INFO("[ovphysx] Attempting to preload internal sidecar...");
    if (loadInternalSidecar()) {
        CARB_LOG_INFO("[ovphysx] Internal sidecar preloaded successfully");
    } else {
        CARB_LOG_INFO("[ovphysx] Internal sidecar preload failed (will load on-demand on first use)");
    }

    // Validate active_cuda_gpus ordinal early so the user gets a fast error at create
    // time rather than at first attach. The actual /physics/cudaDevice write is
    // deferred to ovphysx_ensure_physics_attached() under g_gpuAttachMutex so that
    // it is held atomically with physxSim->attachStage().
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
        // after this call returns. config_entries were already applied above;
        // active_cuda_gpus is consumed at first attach via the parsed
        // active_cuda_ordinals below, so the string view is no longer needed.
        instanceData->create_args.config_entries = nullptr;
        instanceData->create_args.config_entry_count = 0;
        instanceData->create_args.active_cuda_gpus.ptr = nullptr;
        instanceData->create_args.active_cuda_gpus.length = 0;
        // Persist the parsed ordinals (resolved above into requestedOrdinals) only
        // when the caller actually restricted GPU ordinals. requestedOrdinals
        // defaults to {0} for empty input, so gate on the original string to keep
        // "no active_cuda_gpus" distinct from an explicit "0".
        if (args->active_cuda_gpus.ptr && args->active_cuda_gpus.length > 0) {
            instanceData->active_cuda_ordinals = requestedOrdinals;
        }
    } else {
        instanceData->create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    }

    const ovphysx_handle_t handle = ovphysx::internal::allocateOpaqueObjectHandle();
    if (handle == OVPHYSX_INVALID_HANDLE)
        return set_error(OVPHYSX_API_ERROR, "opaque object handle space exhausted");

    CARB_LOG_INFO("[ovphysx] Instance %" PRIu64 " created", handle);

    {
        std::unique_lock<std::shared_mutex> map_lock(g_instances_mutex);
        g_instances[handle] = std::move(instanceData);
    }
    *out_handle = handle;

    // DirectGPU (/physics/suppressReadback) is intentionally host-managed.

    return success();
}

// Process-wide lifecycle: ovphysx_initialize() / ovphysx_shutdown() / the
// init-check in ovphysx_create_instance().
//
// Per the ovphysx threading contract, process-lifecycle calls must be
// serialized by the caller (the Python wrapper does this via
// _PROCESS_LIFECYCLE_LOCK). The g_initialized atomic only enforces matched
// init/shutdown pairing; it is NOT a lock. We intentionally do not guard these
// with g_createInstanceMutex: ovphysx_create_instance() delegates to
// createInstanceInternal(), which already takes that (non-recursive) mutex, so
// checking g_initialized under it here would self-deadlock.
//
// This is safe today because initialize/shutdown are no-op placeholders (they
// only flip the flag). If they ever gain real global setup/teardown, a
// concurrent shutdown-vs-create would become a use-after-free hazard and this
// will need a dedicated lifecycle mutex held across the create path — not the
// create mutex.
OVPHYSX_API ovphysx_result_t ovphysx_initialize(void)
{
    // Reserved process-wide lifecycle slot for future global initialization and OV library API conformance.
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
    // No g_createInstanceMutex here: omni_sdk_physx_destroy has its own
    // internal serialization, and ovphysx_destroy_instance is called from
    // PhysX RAII destructors that can fire on error paths within
    // createInstanceInternal (which holds g_createInstanceMutex) -- taking
    // it here would deadlock.
    ovphysx_api_status_t status = omni_sdk_physx_destroy(handle);
    if (status != OVPHYSX_API_SUCCESS) {
        return {status}; // Don't set error on destroy failure
    }
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

OVPHYSX_API ovphysx_result_t ovphysx_shutdown(void) {
    // Guard: must be called with a matching ovphysx_initialize().
    bool expected = true;
    if (!g_initialized.compare_exchange_strong(expected, false, std::memory_order_acq_rel))
    {
        return set_error(OVPHYSX_API_ERROR, "ovphysx_shutdown called without matching ovphysx_initialize");
    }

    // If instances are still alive, the g_initialized flag is cleared above
    // (so a second shutdown() will error), but handles stay owned by their
    // callers. Static runtime services remain resident until process exit.
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        if (!g_instances.empty()) {
            return success();
        }
    }

    // Drain the direct PhysX runtime while Carbonite's settings/dictionary/log
    // plugins are still resident. omni::physx::runtime::shutdown() tears down
    // OmniPhysX (UJITSO processors, scenes, tensors, foundation runtime). If we
    // skip it, those objects are instead destroyed during C++ static destruction
    // at process exit, where they reach back into an already-torn-down Carbonite
    // (carb.settings -> carb::dictionary::IDictionary fails to resolve) and the
    // process dies with an access violation, preceded by "Leaked processor"
    // UJITSO errors. That is the teardown crash CI's Windows python-runtime and
    // C-sample jobs hit. The drain must happen here on the explicit, terminal
    // ovphysx_shutdown() call -- the per-instance destroy path keeps the runtime
    // resident so device-switch create/destroy/create cycles can reuse it.
    //
    // The Carbonite framework itself is intentionally kept resident for its
    // static process-exit hook; only the direct runtime is drained here.
    omni::physx::runtime::shutdown();

    CARB_LOG_VERBOSE("[ovphysx] Direct runtime shut down; Carbonite framework kept resident until process exit");

    return success();
}


OVPHYSX_API ovphysx_enqueue_result_t ovphysx_reset_stage(ovphysx_handle_t handle) {
    ovphysx_api_status_t wait_status = wait_for_all_pending_ops(handle);
    if (wait_status != OVPHYSX_API_SUCCESS) {
        return set_enqueue_error(wait_status, "Failed to complete pending operations before reset_stage");
    }

    // Gating solely on ovstage_attached is complete, not just a special case:
    // attachedStageId is set/cleared exclusively by ovphysx_attach_ovstage() /
    // ovphysx_detach_ovstage(), always together with ovstage_attached (see
    // those two functions) -- there is no live attach path that leaves a
    // handle with attachedStageId != 0 while ovstage_attached is false. So
    // ovphysx_detach_ovstage() is the only place gpu_warmup_done/
    // first_step_done need clearing here; the stageId-keyed fallback in
    // omni_sdk_physx_unload_usd() (full instance teardown only) is a
    // defensive safety net, not a second reachable attach flow.
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
    // Reject negative / non-finite dt up front so a bad value never advances
    // (and poisons) the internal sim-time counter.
    if (step_dt < 0.0f || !std::isfinite(step_dt))
        return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");

    // Reject a stage-less handle before ever reaching simulate(). Without this,
    // omni_sdk_physx_simulate_instance()'s ensure_physics_attached() call treats
    // "no stage" as a trivial success and falls through to physxSim->simulate()
    // unconditionally -- and IPhysxSimulation is a process-wide singleton, so
    // that call would silently advance whatever OTHER handle's stage happens
    // to be attached, while only THIS handle's first_step_done/gpu_warmup_done
    // get set (not the actual stage owner's), letting clone() on the real
    // owner pass its after-step guard despite the owner's stage having
    // genuinely been stepped. ovphysx_step_sync()/_step_n_sync() already guard
    // on this; this closes the same hole for the async path.
    {
        std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);
        InstanceData* instance = get_instance_ptr(handle);
        if (!instance)
            return set_enqueue_error(OVPHYSX_API_ERROR, "Invalid handle");
        const bool hasPhysicsStage = (instance->attachedStageId != 0) || instance->ovstage_attached;
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

    // Call internal simulate with the counter value as current time.
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
            // Cache last dt for contact read functions (force = impulse / dt).
            // Clamp to a tiny positive value so we never divide by zero
            // (a zero-dt step produces zero impulses, and 0/eps ~ 0).
            instance->last_step_dt = (step_dt > 0.0f) ? step_dt : 1.0f;
            // Advance the internal counter once the step is successfully
            // enqueued. (Async: this is dispatch success, not fetch
            // completion — the counter reflects the step that was issued.)
            instance->sim_time = next_time;
            // Mark GPU warmup as done for GPU-capable processes so that clone()
            // can guard against post-step cloning (which would corrupt GPU buffers).
            // Skipped for CPU-only processes since there are no GPU buffers to corrupt.
            if (instance->attachedStageId != 0 && !isProcessGpuDisabled()) {
                instance->gpu_warmup_done.store(true, std::memory_order_release);
                instance->gpu_warmup_stage_id.store(instance->attachedStageId, std::memory_order_release);
            }
            // Mark first-step-done in both CPU and GPU mode (unlike gpu_warmup_done,
            // which is GPU-only) so clone()'s after-step precondition is enforced the
            // same way in both modes. Still gated on attachedStageId != 0 -- this
            // handle's own stage, not the process-global runtime -- so a handle with
            // no attached stage is never falsely marked as having stepped.
            if (instance->attachedStageId != 0) {
                instance->first_step_done.store(true, std::memory_order_release);
            }
        }
    }

    ovphysx_op_index_t op_index = ovphysx::async::register_operation(handle, event);

    return enqueue_success(op_index);
}

// Fast synchronous step+wait that bypasses the async event machinery.
// Equivalent to ovphysx_step() followed immediately by wait_op(), but
// uses a single lock acquisition and avoids the AsyncEventManager overhead
// (~0.88ms per step in the common synchronous case).
OVPHYSX_API ovphysx_result_t ovphysx_step_sync(ovphysx_handle_t handle,
                                                float step_dt) {
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
    const bool hasPhysicsStage = (instance->attachedStageId != 0) || instance->ovstage_attached;
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

    // Dispatch GPU work (returns almost immediately - ~0.2ms).
    physxSim->simulate(step_dt, current_time);

    if (!isProcessGpuDisabled()) {
        instance->gpu_warmup_done.store(true, std::memory_order_release);
        instance->gpu_warmup_stage_id.store(instance->attachedStageId, std::memory_order_release);
    }
    // Mark first-step-done in both CPU and GPU mode (unlike gpu_warmup_done,
    // which is GPU-only). Gated on attachedStageId != 0 like the async
    // ovphysx_step() path: hasPhysicsStage above can be satisfied by
    // ovstage_attached alone (attachedStageId == 0), and such a handle must not
    // be marked as having stepped its own USD stage.
    if (instance->attachedStageId != 0) {
        instance->first_step_done.store(true, std::memory_order_release);
    }

    // Release map lock while waiting for GPU to avoid holding it during the
    // 3-4ms fetchResults() blocking call.
    map_lock.unlock();

    // Wait for GPU (3-4ms).
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
    if (n_steps <= 0)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "n_steps must be > 0");
    if (step_dt < 0.0f || !std::isfinite(step_dt))
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid step_dt: must be a finite value >= 0.0");

    std::shared_lock<std::shared_mutex> map_lock(g_instances_mutex);

    InstanceData* instance = get_instance_ptr(handle);
    if (!instance)
        return set_error(OVPHYSX_API_ERROR, "Invalid handle");

    auto physxSim = instance->carbonite->getPhysxSimulation();
    const bool hasPhysicsStage = (instance->attachedStageId != 0) || instance->ovstage_attached;
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

    // GPU-capability is fixed for the process lifetime; resolve it once rather than
    // re-reading the env var on every iteration of the batch.
    const bool markGpuWarmup = !isProcessGpuDisabled();

    for (int32_t i = 0; i < n_steps; ++i) {
        const float sim_time = base_time + i * step_dt;

        physxSim->simulate(step_dt, sim_time);

        if (markGpuWarmup) {
            instance->gpu_warmup_done.store(true, std::memory_order_release);
            instance->gpu_warmup_stage_id.store(instance->attachedStageId, std::memory_order_release);
        }
        // Mark first-step-done in both CPU and GPU mode, gated on attachedStageId
        // != 0 like gpu_warmup_done above -- this handle's own stage, so a handle
        // without one attached is never falsely marked as having stepped.
        if (instance->attachedStageId != 0) {
            instance->first_step_done.store(true, std::memory_order_release);
        }

        map_lock.unlock();
        physxSim->fetchResults();
        map_lock.lock();

        instance = get_instance_ptr(handle);
        if (!instance)
            return set_error(OVPHYSX_API_ERROR, "Handle invalidated during fetchResults");

        // No ovphysx → ovstage write-back: results are consumed via the read /
        // tensor-binding API; the application owns writing them back to ovstage.
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
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
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
    if (!value_out || !out_required_size || key < 0 || key >= OVPHYSX_CONFIG_STRING_COUNT)
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "Invalid arguments");
    auto* framework = carb::getFramework();
    auto* settings = framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings) return set_error(OVPHYSX_API_ERROR, "Settings interface not available");
    char* mutable_buffer = const_cast<char*>(value_out->ptr);
    size_t required_size = 0;
    if (!getSettingValueAsString(settings, s_stringKeyPaths[key], mutable_buffer, static_cast<uint32_t>(value_out->length), &required_size)) {
        *out_required_size = 0;
        return set_error(OVPHYSX_API_NOT_FOUND, "Config value not found");
    }
    *out_required_size = required_size;
    if (required_size > value_out->length) return set_error(OVPHYSX_API_BUFFER_TOO_SMALL, "Buffer too small");
    value_out->length = strlen(value_out->ptr);
    return {OVPHYSX_API_SUCCESS};
}

OVPHYSX_API ovphysx_result_t ovphysx_attach_ovstage(ovphysx_handle_t handle,
                                                     ovstage_instance_t* stage,
                                                     ovstage_ordinal_t read_ordinal) {
    if (!stage) {
        return set_error(OVPHYSX_API_INVALID_ARGUMENT, "attach_ovstage: stage is null");
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
    if (instanceShared->attachedStageId != 0 || instanceShared->ovstage_attached) {
        return set_error(OVPHYSX_API_ERROR,
                         "attach_ovstage: a stage is already attached; detach or reset before attaching ovstage");
    }

    omni::physx::IPhysxSimulation* physxSim =
        instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->attachOvstage) {
        return set_error(OVPHYSX_API_ERROR,
                         "attach_ovstage: IPhysxSimulation::attachOvstage is unavailable");
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

    // The caller owns the sealed read ordinal and passes it explicitly; the
    // initial scene parse reads at this ordinal.
    const bool attached = physxSim->attachOvstage(&instanceShared->ovstage_attach_payload, read_ordinal);
    if (!attached) {
        instanceShared->ovstage_attach_payload = OvstageAttachPayload{};
        return set_error(OVPHYSX_API_ERROR,
                         "attach_ovstage: IPhysxSimulation::attachOvstage failed");
    }

    const int64_t stageId = physxSim->getAttachedStage
        ? static_cast<int64_t>(physxSim->getAttachedStage())
        : 0;

    instanceShared->attachedStageId = stageId;
    instanceShared->ovstage_attached = true;
    instanceShared->resetStageFlags(/*physicsAttached=*/stageId != 0);
    if (stageId != 0) {
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
                                                   const float* parent_transforms,
                                                   const uint32_t* env_ids) {
    // Clone the source subtree to the target paths via the PhysX SDK replicator (routed
    // through IPhysxSimulation::cloneEnvironments). Only path strings + a flat
    // [num_target_paths * 7] transform array cross the C ABI (no USD types).
    // parent_transforms[i] positions copy i's parent (copy = transform[i] *
    // inverse(source_parent) * body), NULL co-locates on the source. env_ids[i] names
    // copy i's logical environment (stable across calls), NULL = per-call numbering.

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

    // Enforce the same clone-before-step contract in CPU and GPU mode. gpu_warmup_done
    // catches explicit GPU warmup; first_step_done catches every stepping entry point.
    // Use a synchronous error with no op so reset_stage() remains a valid recovery path.
    if (instanceShared->gpu_warmup_done.load(std::memory_order_acquire) ||
        instanceShared->first_step_done.load(std::memory_order_acquire)) {
        return set_enqueue_error(OVPHYSX_API_INVALID_ARGUMENT,
                                 "clone: must be called before warmup_gpu() and the first step(). "
                                 "Cloning after GPU warmup reallocates buffers and silently corrupts "
                                 "simulation state. Call reset_stage() to clone after warmup.");
    }

    omni::physx::IPhysxSimulation* physxSim =
        instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
    if (!physxSim || !physxSim->cloneEnvironments) {
        return set_enqueue_error(OVPHYSX_API_ERROR, "clone: IPhysxSimulation::cloneEnvironments is unavailable");
    }

    // env-id cross-environment collision filtering is a per-process setting (default on); with
    // explicit transforms the copies are already physically separated, so it is an optional add-on.
    bool useEnvIds = true;
    if (carb::Framework* fw = carb::getFramework()) {
        if (carb::settings::ISettings* settings = fw->tryAcquireInterface<carb::settings::ISettings>()) {
            settings->setDefaultBool("/ovphysx/clone/useEnvIds", true);
            useEnvIds = settings->getAsBool("/ovphysx/clone/useEnvIds");
        }
    }

    // Views are not guaranteed null-terminated; the runtime entry takes C strings. Build a
    // null-terminated source path + a stable array of target C-string pointers.
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
        // The length-tagged view can carry an embedded NUL; the seam takes a C string, so
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
    // every environment id < 1<<24 (setEnvironmentID), so the caller id must be < 0x00FFFFFF;
    // a larger value would silently make setEnvironmentID fail (body collides with all).
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
                                             num_target_paths, parent_transforms, env_ids, useEnvIds);
    } catch (const std::exception& e) {
        return set_enqueue_error(OVPHYSX_API_ERROR, std::string("clone: cloneEnvironments threw: ") + e.what());
    } catch (...) {
        return set_enqueue_error(OVPHYSX_API_ERROR, "clone: cloneEnvironments threw a non-std exception");
    }
    if (!cloned) {
        return set_enqueue_error(OVPHYSX_API_ERROR,
                                 "clone: cloneEnvironments failed (a target may already be populated with "
                                 "physics, or the source subtree is invalid) -- see the log for the specific path");
    }

    // A tensor backend created before clone() cached the pre-clone actor population and
    // buffer sizes. Invalidate it after a successful clone so the next binding rebuilds
    // against the complete population; existing pre-clone views are stale by definition.
    const int64_t stageId = instanceShared->attachedStageId;
    if (stageId != 0) {
        if (omni::physics::tensors::TensorApi* tensorApi =
                omni::physx::runtime::tryGetTensorApiInterface()) {
            if (tensorApi->resetStage)
                tensorApi->resetStage(stageId);
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

        // ovx_primpath_t handles are owned by the attached Stage's path
        // dictionary. Drop the process-global visualization scope before the
        // runtime detaches (and the caller may destroy that dictionary).
        clearVisualizationScopeTokens();

        // Release SDF views before PhysX/tensor teardown so native ISdfShapeView
        // objects do not outlive the per-stage SimulationBackend data.
        ovphysx_sdf_view_cleanup_instance(instanceShared.get());

        omni::physx::IPhysxSimulation* physxSim =
            instanceShared->carbonite ? instanceShared->carbonite->getPhysxSimulation() : nullptr;
        if (physxSim && physxSim->detachStage) {
            physxSim->detachStage();
        }


        if (stageId != 0) {
            // Release the tensor SimulationBackend's per-stage data so a later reattach starts
            // clean; stale views/data would otherwise persist across reset_stage / reattach.
            if (omni::physics::tensors::TensorApi* tensorApi =
                    omni::physx::runtime::tryGetTensorApiInterface()) {
                if (tensorApi->resetStage)
                    tensorApi->resetStage(stageId);
            }
            markStageDetached(stageId, "detach_ovstage");
        }
        instanceShared->attachedStageId = 0;
        instanceShared->ovstage_attached = false;
        instanceShared->ovstage_attach_payload = OvstageAttachPayload{};
        instanceShared->resetStageFlags();
    }

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
        // The user task stored its error in TLS via set_error(); retrieve it
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
                                                     uint64_t timeout_ns,
                                                     ovphysx_op_wait_result_t* out_wait_result) {
    // Clear per-op errors at the start of each wait_op call
    tls_error().op_errors.clear();

    // Fast path: for the RL hot loop (step -> wait_op -> reads/writes -> step),
    // the only tracked op can skip the generic get_pending_ops/event machinery
    // and go straight to simulation sync.
    if (op_index != OVPHYSX_OP_INDEX_ALL) {
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
            // This IS the simulation event -- sync directly
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

    auto timeout_duration = clamp_timeout_ns(timeout_ns);
    auto start_time = std::chrono::steady_clock::now();

    // Collect failed op indices; error messages go into TLS op_errors map
    std::vector<ovphysx_op_index_t> collected_error_indices;

    ovphysx_op_index_t lowest_pending = 0;
    for (ovphysx_op_index_t pending_op : pending_ops) {
        // Calculate remaining timeout (allow zero to still poll once)
        uint64_t remaining_timeout_ns = 0;
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (timeout_duration.count() == 0 || elapsed >= timeout_duration) {
            remaining_timeout_ns = 0;  // immediate poll only
        } else {
            remaining_timeout_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(timeout_duration - elapsed).count());
        }

        async_event_handle_t event = ovphysx::async::get_event_for_op(handle, pending_op);
        // event may be 0 for CUDA-only ops (tensor binding async). wait_on_single_event
        // handles CUDA events first. For completed ops, it returns SUCCESS.

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

    return success();
}

OVPHYSX_API ovphysx_string_t ovphysx_get_last_error(void) {
    auto& err = tls_error().last_error;
    if (err.empty()) return {nullptr, 0};
    return {err.c_str(), err.size()};
}

OVPHYSX_API ovphysx_string_t ovphysx_get_last_op_error(ovphysx_op_index_t op_index) {
    auto& op_errors = tls_error().op_errors;
    auto it = op_errors.find(op_index);
    if (it == op_errors.end() || it->second.empty()) return {nullptr, 0};
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

// Private C API: load the internal tensor plugins used by TensorBindingsAPI.
// Exposed symbol is needed for ctypes, but not in the public header.
// Uses the same flat plugins/ directory as CarboniteLoader.
extern "C" OVPHYSX_API int32_t ovphysx_load_tensor_plugins(void)
{
    // Load tensor plugins once after success, but allow retry after early
    // failures. Python can ask for TensorApi before ovphysx has loaded config;
    // std::call_once would permanently poison that process.
    static std::mutex s_tensorPluginsMutex;
    static bool s_tensorPluginsLoaded = false;

    std::lock_guard<std::mutex> lock(s_tensorPluginsMutex);
    if (s_tensorPluginsLoaded)
        return OVPHYSX_API_SUCCESS;

    carb::Framework* framework = carb::getFramework();
    if (!framework)
    {
        CARB_LOG_ERROR("[TensorPlugins] Carbonite framework unavailable");
        return OVPHYSX_API_ERROR;
    }

    // Get plugins directory (shared utility with CarboniteLoader)
    std::string pluginsDirStr = omni::sdk::usd_version::getPluginsDirectory();
    if (pluginsDirStr.empty() || !std::filesystem::exists(pluginsDirStr))
    {
        CARB_LOG_ERROR("[TensorPlugins] Plugins directory not found: %s", pluginsDirStr.c_str());
        return OVPHYSX_API_ERROR;
    }

    std::vector<std::string> searchPaths = { pluginsDirStr };
    CARB_LOG_INFO("[TensorPlugins] Using plugins directory: %s", pluginsDirStr.c_str());

    std::vector<std::string> preloadPaths = { pluginsDirStr };
    try
    {
        std::filesystem::path pluginsPath(pluginsDirStr);
        std::filesystem::path libsPath = pluginsPath.parent_path().parent_path() / "ovphysx.libs";
        if (std::filesystem::exists(libsPath))
        {
            preloadPaths.push_back(libsPath.string());
            CARB_LOG_INFO("[TensorPlugins] Using libs directory: %s", libsPath.string().c_str());
        }
    }
    catch (const std::exception& e)
    {
        CARB_LOG_WARN("[TensorPlugins] Failed to probe ovphysx.libs: %s (falling back to plugins directory)", e.what());
    }

    std::vector<const char*> searchPathsC;
    searchPathsC.reserve(searchPaths.size());
    for (const std::string& p : searchPaths)
        searchPathsC.push_back(p.c_str());

#ifdef _WIN32
    // Windows: Add all search paths to the process PATH so dependent DLLs can be found
    // This is required because Windows doesn't use rpath like Linux
    static bool s_tensorPluginPathConfigured = false;
    if (!s_tensorPluginPathConfigured)
    {
        std::string path_additions;
        for (const std::string& search_path : preloadPaths)
        {
            if (!path_additions.empty())
                path_additions += ";";
            path_additions += search_path;
        }

        if (!path_additions.empty())
        {
            const char* current_path = std::getenv("PATH");
            std::string new_path = path_additions;
            if (current_path && current_path[0] != '\0')
            {
                new_path += ";";
                new_path += current_path;
            }

            if (_putenv_s("PATH", new_path.c_str()) != 0)
            {
                CARB_LOG_WARN("[TensorPlugins] WARNING: Failed to update PATH environment variable");
            }
            else
            {
                s_tensorPluginPathConfigured = true;
                CARB_LOG_INFO("[TensorPlugins] Added %zu directories to PATH for DLL resolution", preloadPaths.size());
            }
        }
    }
#else
    // Linux: Pre-load additional tensor libraries with RTLD_GLOBAL
    // Core libraries are already loaded by CarboniteLoader during initialization
    const std::vector<std::string>* libs_to_preload = omni::sdk::usd_version::getPreloadLibrariesLinux();
    if (!libs_to_preload)
    {
        CARB_LOG_ERROR("[TensorPlugins] FATAL: Config not available. Call ovphysx_create_instance() first.");
        return OVPHYSX_API_ERROR;
    }

    if (libs_to_preload->empty())
    {
        CARB_LOG_INFO("[TensorPlugins] No additional tensor libraries to preload");
    }
    else
    {
        CARB_LOG_INFO("[TensorPlugins] Preloading %zu additional tensor libraries from config", libs_to_preload->size());

        int loaded_count = 0;
        // Resolve library path by exact name first, then by stem-*.so suffix.
        // This handles versioned/hashed library filenames without computing any hash.
        auto resolve_versioned_lib_path = [](const std::vector<std::string>& paths, const std::string& lib_name) -> std::string
        {
            for (const std::string& search_path : paths)
            {
                std::string resolved = omni::sdk::internal::findLibPath(
                    search_path,
                    lib_name.c_str(),
                    [&lib_name](const std::filesystem::path& dir, const char* libName, const std::filesystem::filesystem_error& e)
                    {
                        CARB_LOG_WARN("[TensorPlugins] Failed to scan %s for %s: %s",
                                      dir.string().c_str(), libName, e.what());
                    });
                if (!resolved.empty())
                    return resolved;
            }
            return {};
        };

        bool preloadSucceeded = true;
        for (const std::string& lib_name : *libs_to_preload)
        {
            const std::string resolved = resolve_versioned_lib_path(preloadPaths, lib_name);
            if (!resolved.empty())
            {
                void* handle = dlopen(resolved.c_str(), RTLD_NOW | RTLD_GLOBAL);
                if (handle)
                {
                    loaded_count++;
                }
                else
                {
                    preloadSucceeded = false;
                    CARB_LOG_WARN("[TensorPlugins] Failed to pre-load %s: %s",
                                  resolved.c_str(), dlerror());
                }
            }
            else
            {
                preloadSucceeded = false;
                CARB_LOG_WARN("[TensorPlugins] Library not found in any search path: %s", lib_name.c_str());
            }
        }

        CARB_LOG_INFO("[TensorPlugins] Pre-loaded %d/%zu additional tensor libraries",
                      loaded_count, libs_to_preload->size());
        if (!preloadSucceeded)
        {
            CARB_LOG_ERROR("[TensorPlugins] Tensor library preloading failed. Cannot load tensor plugins.");
            return OVPHYSX_API_ERROR;
        }
    }
#endif

    // Load plugins from config (config was already loaded in ovphysx_create_instance)
    const std::vector<omni::sdk::usd_version::PluginConfig>* plugins = omni::sdk::usd_version::getPlugins();
    if (!plugins)
    {
        CARB_LOG_ERROR("[TensorPlugins] FATAL: Config not available. Cannot determine which plugins to load.");
        CARB_LOG_ERROR("[TensorPlugins] This should not happen - config should be loaded during ovphysx_create_instance().");
        return OVPHYSX_API_ERROR;
    }

    if (plugins->empty())
    {
        CARB_LOG_INFO("[TensorPlugins] No plugins configured to load");
        s_tensorPluginsLoaded = true;
        return OVPHYSX_API_SUCCESS;
    }

    carb::PluginLoadingDesc desc = carb::PluginLoadingDesc::getDefault();
    desc.searchPaths = searchPathsC.data();
    desc.searchPathCount = (uint32_t)searchPathsC.size();
    desc.excludedFileWildcards = nullptr;
    desc.excludedFileWildcardCount = 0;
    
    for (const auto& plugin : *plugins)
    {
        CARB_LOG_INFO("[TensorPlugins] Loading plugin: %s (required=%s)", 
                      plugin.name.c_str(), plugin.required ? "true" : "false");

        // Verify the plugin binary exists in at least one search path before loading.
        // Some plugins do not expose IExt; the existence check prevents us from
        // silently succeeding when the binary is missing.
#if CARB_PLATFORM_WINDOWS
        std::string pluginFilename = plugin.name + ".dll";
#else
        std::string pluginFilename = "lib" + plugin.name + ".so";
#endif

        bool pluginBinaryFound = false;
        for (const std::string& search_path : searchPaths)
        {
            const auto candidate = std::filesystem::path(search_path) / pluginFilename;
            if (std::filesystem::exists(candidate))
            {
                pluginBinaryFound = true;
                break;
            }
        }

        if (!pluginBinaryFound)
        {
            CARB_LOG_ERROR("[TensorPlugins] Plugin binary not found: %s", pluginFilename.c_str());
            if (plugin.required)
            {
                return OVPHYSX_API_ERROR;
            }
            continue;
        }
        
        const char* plugin_name = plugin.name.c_str();
        desc.loadedFileWildcards = &plugin_name;
        desc.loadedFileWildcardCount = 1;
        
        framework->loadPlugins(desc);
        
        // Force plugin initialization by acquiring its IExt interface when available.
        // Not all plugins expose IExt (some register their own interfaces only).
        // Treat absence as non-fatal after the binary check.
        auto* ext = framework->tryAcquireInterface<omni::ext::IExt>(plugin.name.c_str());
        if (!ext)
        {
            CARB_LOG_WARN("[TensorPlugins] %s loaded (no omni::ext::IExt interface exposed)", plugin.name.c_str());
        }
    }

    s_tensorPluginsLoaded = true;

    return OVPHYSX_API_SUCCESS;
}

// Log capture API and logging configuration are implemented in LogManager.cpp
