// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "ovphysx/ovphysx.h"
#include "CarboniteLoader/CarboniteLoader.hpp"
#include "AsyncEventManager/AsyncEventManager.hpp" // async_event_handle_t
#include "internal/sdk/ovphysxObjectHandleAllocator.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <string>
#include <mutex>
#include <shared_mutex>
#include <atomic>
#include <cstring>
#include <cstdint>
#include <cstddef> // offsetof

// AttachHandle is a value type, so it needs the real declaration. ISimulationView.h
// brings kMaxPathPatternComponentLength and forward-declares the view classes.
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physics/tensors/ISimulationView.h>

struct TensorBindingState
{
    // Identity of the attach this binding was made against, NOT a USD stage id
    // (ADR-0013). A stage id cannot serve here: it is 0 both for a stageless
    // attach and for "detached", so the staleness check below could not tell a
    // live binding from one whose attach had been torn down and replaced.
    omni::physics::tensors::AttachHandle attachHandle = omni::physics::tensors::kNoAttach;
    ovphysx_tensor_type_t tensorType = OVPHYSX_TENSOR_INVALID;
    std::string pattern;
    omni::physics::tensors::ISimulationView* simView = nullptr;
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
    uint64_t usdStageId = 0;
};

// Frozen ABI layout for 64-bit pointers (LP64/LLP64). Must match OvstageAttach.
static_assert(sizeof(OvstageAttachPayload) == 24, "OvstageAttachPayload ABI size drifted from OvstageAttach");
static_assert(offsetof(OvstageAttachPayload, instance) == 0, "OvstageAttachPayload::instance offset drifted");
static_assert(offsetof(OvstageAttachPayload, dict) == 8, "OvstageAttachPayload::dict offset drifted");
static_assert(offsetof(OvstageAttachPayload, usdStageId) == 16, "OvstageAttachPayload::usdStageId offset drifted");

// Lock ordering for InstanceData (to prevent deadlocks):
//   1. g_instances_mutex (global, outermost - shared or exclusive)
//   2. InstanceData::tensor_binding_mutex (per-instance)
//   3. InstanceData::op_tracking_mutex (per-instance)
//   4. InstanceData::simulationMutex (per-instance, innermost)
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

struct InstanceData
{
    std::unique_ptr<ovphysx::CarboniteLoader> carbonite;
    ovphysx_create_args create_args;
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
    // (reused across batches) would create a duplicate object under an existing path, so it is
    // rejected. Cleared on stage teardown (detach / unload), so a fresh attach starts empty. Guarded
    // by the single-instance serialization contract.
    std::unordered_set<std::string> cloned_target_paths;

    // Async simulation state
    float pendingElapsedTime = 0.0f;
    float pendingCurrentTime = 0.0f;
    // Last successful step_dt from any stepping entry point, used by contact
    // read functions (force = impulse / dt). Initialized to 1.0f so pre-step
    // reads are safe. Clamped to positive on write so that a zero-dt step never
    // causes division by zero in the contact force conversion. Thread safety
    // relies on the single-instance serialization contract.
    float last_step_dt = 1.0f;
    // Internal simulation-time counter: the simulation time at the start of the
    // next step. Initialized to 0.0 at construction, advanced by dt per
    // step/step_sync and by n*dt per step_n_sync. Fed to simulate() for
    // timestamps and not exposed through the public API. Guarded by the same
    // single-instance serialization contract as last_step_dt.
    float sim_time = 0.0f;
    async_event_handle_t pendingSimulationEvent = 0;
    std::mutex simulationMutex;  // Guards simulation state. See lock ordering above.
    
    // Per-instance async operation tracking. op_to_event holds the
    // CPU/AsyncEventManager events. Each op_index is single-use: after waiting,
    // the event is destroyed and removed from the map.
    std::atomic<ovphysx_op_index_t> next_op_index{1};
    std::unordered_map<ovphysx_op_index_t, async_event_handle_t> op_to_event;
    // Highest op_index consumed by wait_for_all_pending_ops. A missing index at
    // or below this watermark can be acknowledged once by a later explicit wait.
    // Protected by op_tracking_mutex.
    ovphysx_op_index_t last_internally_synced_op_index = 0;
    // Highest op_index consumed by an explicit wait_op call. The wait processes
    // indices in order and stops at the first pending timeout, so consumed
    // indices always form a prefix. This takes precedence over the internal-sync
    // watermark. Protected by op_tracking_mutex.
    ovphysx_op_index_t last_explicitly_consumed_op_index = 0;
    std::mutex op_tracking_mutex;  // Guards op maps. See lock ordering above.

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
    };
    std::unordered_map<ovphysx_sdf_view_handle_t, SdfViewState> sdf_views;

    // Scene query internal hit buffer, valid until the next scene query call.
    // Not thread-safe: concurrent scene queries on the same instance are a data race.
    // Callers serialize access per the single-threaded instance contract.
    std::vector<ovphysx_scene_query_hit_t> sceneQueryHitBuffer;

    // Warmup tracking: requires at least one simulate() before tensor reads (CPU or GPU).
    // Reset when stage changes. Atomic for thread-safe double-checked locking in read path.
    std::atomic<bool> warmup_done{false};
    // Attach the warmup was performed for. Keyed by attach handle, not stage id:
    // a stage id of 0 means both "stageless attach" and "nothing attached", so a
    // fresh attach would inherit the previous one's warmup flag.
    std::atomic<omni::physics::tensors::AttachHandle> warmup_attach_handle{
        omni::physics::tensors::kNoAttach};

    // Whether the first step() (any of ovphysx_step/_sync/_n_sync) has run for
    // the current stage, in any mode (CPU or GPU). Set unconditionally so clone()'s
    // after-step guard fires even when warmup() was not called explicitly. Reset
    // alongside warmup_done wherever that flag is cleared.
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
    // Fast-path flag for tensor read/write. Set to true after wait_op/sync
    // completes with no pending ops. Cleared when a new step is enqueued.
    // When true, read/write skips wait_all_pending_internal, which acquires
    // two mutexes and allocates a vector.
    std::atomic<bool> all_ops_synced{false};
    
    InstanceData() = default;
    
    ~InstanceData();
    
    // Not copyable due to unique_ptr and mutex.
    InstanceData(const InstanceData&) = delete;
    InstanceData& operator=(const InstanceData&) = delete;
    
    // Not movable either. InstanceData is stored in a shared_ptr, so only the
    // shared_ptr moves.
    InstanceData(InstanceData&&) = delete;
    InstanceData& operator=(InstanceData&&) = delete;
};

extern std::unordered_map<ovphysx_handle_t, std::shared_ptr<InstanceData>> g_instances;
extern std::shared_mutex g_instances_mutex;

// True when this process must never touch the GPU: either ovphysx_set_cpu_mode(true)
// forced CPU-only mode or OVPHYSX_DISABLE_GPU is active (live before initialize /
// latched at ovphysx_initialize). Defined in ovphysx.cpp. Use this rather than
// checking OVPHYSX_DISABLE_GPU directly so both inputs are honored.
bool isProcessGpuDisabled();

// Waits for all pending operations (stream-ordered execution). Returns
// OVPHYSX_API_SUCCESS when all operations completed, or an error status.
ovphysx_api_status_t omni_sdk_physx_wait_all_pending_internal(ovphysx_handle_t handle);

// Starts a simulation step and returns without waiting for it. Used by
// ovphysx_step() and auto-warmup in tensor binding.
ovphysx_api_status_t omni_sdk_physx_simulate_instance(ovphysx_handle_t handle, float elapsedTime, float currentTime);

// Looks up an instance by handle, taking g_instances_mutex internally. Returns
// nullptr if the handle is not found. The returned shared_ptr keeps the instance
// alive even if another thread calls ovphysx_destroy_instance.
inline std::shared_ptr<InstanceData> get_instance(ovphysx_handle_t handle) {
    std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
    auto it = g_instances.find(handle);
    return (it != g_instances.end()) ? it->second : nullptr;
}

// Raw pointer version for callers that already hold g_instances_mutex. The
// returned pointer becomes invalid after releasing the lock.
inline InstanceData* get_instance_ptr(ovphysx_handle_t handle) {
    auto it = g_instances.find(handle);
    return (it != g_instances.end()) ? it->second.get() : nullptr;
}


// Lazy physics-attach helper (ovphysx.cpp). Triggers attachStage() if not
// yet done for the current stage.
ovphysx_api_status_t ovphysx_ensure_physics_attached(ovphysx_handle_t handle);

// Tensor binding cleanup (ovphysxTensorBinding.cpp)
void ovphysx_tensor_binding_cleanup_instance(InstanceData* instance);

// Contact binding cleanup (ovphysxContactBinding.cpp)
void ovphysx_contact_binding_cleanup_instance(InstanceData* instance);

// SDF view cleanup (ovphysxSdfView.cpp)
void ovphysx_sdf_view_cleanup_instance(InstanceData* instance);

// Warmup helper (ovphysxTensorBinding.cpp), also callable from contact/SDF binding.
ovphysx_result_t ovphysx_warmup_if_needed(ovphysx_handle_t handle, bool is_explicit_call);

// ============================================================================
// Thread-local error storage (TLS error model)
// ============================================================================

struct ThreadLocalError {
    std::string last_error;                                        // general API error
    std::unordered_map<ovphysx_op_index_t, std::string> op_errors; // per-op errors from wait_op
};

inline ThreadLocalError& tls_error() {
    thread_local ThreadLocalError t_error;
    return t_error;
}

inline ovphysx_result_t set_error(ovphysx_api_status_t status, const char* msg) {
    tls_error().last_error = msg ? msg : "";
    return {status};
}

inline ovphysx_result_t set_error(ovphysx_api_status_t status, const std::string& msg) {
    tls_error().last_error = msg;
    return {status};
}

inline ovphysx_result_t success() {
    tls_error().last_error.clear();
    return {OVPHYSX_API_SUCCESS};
}

inline ovphysx_enqueue_result_t set_enqueue_error(ovphysx_api_status_t status, const char* msg) {
    tls_error().last_error = msg ? msg : "";
    return {status, 0};
}

inline ovphysx_enqueue_result_t set_enqueue_error(ovphysx_api_status_t status, const std::string& msg) {
    tls_error().last_error = msg;
    return {status, 0};
}

inline ovphysx_enqueue_result_t enqueue_success(ovphysx_op_index_t op_index) {
    tls_error().last_error.clear();
    return {OVPHYSX_API_SUCCESS, op_index};
}

inline ovphysx_string_t make_error(const char* msg) {
    tls_error().last_error = msg ? msg : "";
    return {tls_error().last_error.c_str(), tls_error().last_error.size()};
}

inline ovphysx_string_t make_error(const std::string& msg) {
    return make_error(msg.c_str());
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
