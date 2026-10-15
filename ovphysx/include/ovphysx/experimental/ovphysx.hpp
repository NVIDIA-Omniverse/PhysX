// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-TASK-001
// @covers AC-1 AC-4 AC-5

// @implements REQ-CAPI-ASYNC-001
// @covers AC-2

// @implements REQ-CAPI-INSTANCE-SCOPE-001
// @covers AC-1 AC-4

/** @cond PLC
 * @implements REQ-CAPI-OVD-NAMES-001
 * @covers AC-1
 * @implements REQ-CAPI-PHYSXPTR-001
 * @covers AC-5
 * @implements REQ-CAPI-LEGACY-001
 * @covers AC-7
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1
 * @endcond
 */

#ifndef OVPHYSX_EXPERIMENTAL_OVPHYSX_HPP
#define OVPHYSX_EXPERIMENTAL_OVPHYSX_HPP

//
// C++17 RAII wrapper for the ovphysx instance handle.
// Provides automatic cleanup and thin C++ wrappers for the C API functions.
//

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysx/ovphysx_export.h"
#include "ovphysx/experimental/Helpers.hpp"
#include "ovphysx/experimental/TensorBinding.hpp"

namespace ovphysx {

/**
 * @brief Safe wrapper for ovphysx_create_args.
 *
 * Default-constructs to OVPHYSX_CREATE_ARGS_DEFAULT (empty active_cuda_gpus,
 * no config entries, empty bundled deps path). Use setters to override individual
 * fields before passing to PhysX::create().
 *
 * Callers do not need to touch ovphysx_create_args directly. This class
 * guarantees all fields are initialized.
 */
// C4251: STL members in DLL-exported class. These are private implementation
// details accessed only through the public API, so the warning is benign.
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4251)
#endif

class OVPHYSX_API CreateArgs {
public:
    CreateArgs();
    CreateArgs(const CreateArgs&);
    CreateArgs& operator=(const CreateArgs&);
    CreateArgs(CreateArgs&&) noexcept;
    CreateArgs& operator=(CreateArgs&&) noexcept;

    /// @param gpus  Comma-separated CUDA device ordinals, e.g. "0", "0,1,2", "1,2".
    ///              See active_cuda_gpus on ovphysx_create_args for supported patterns.
    ///              CreateArgs copies the string internally, so the caller does not need
    ///              to keep the argument alive after this call returns.
    void setActiveCudaGpus(const std::string& gpus);
    /// @param path  Bundled deps path.
    ///              CreateArgs copies the string internally, so the caller does not need
    ///              to keep the argument alive after this call returns.
    void setBundledDepsPath(const std::string& path);

    /// @param entries  Pointer to an array of config entries. The caller must keep this array
    ///                 valid until PhysX::create() returns. CreateArgs does not copy the data.
    /// @param count    Number of entries in the array.
    void setConfigEntries(const ovphysx_config_entry_t* entries, uint32_t count);

    /// Returns a const reference to the underlying ovphysx_create_args.
    /// The reference is valid only for the lifetime of this CreateArgs object.
    const ovphysx_create_args& cArgs() const;

private:
    std::string m_activeCudaGpus;
    std::string m_bundledDepsPath;
    ovphysx_create_args m_args;
};

#ifdef _MSC_VER
#pragma warning(pop)
#endif

/**
 * @brief RAII wrapper for ovphysx_instance_handle_t
 *
 * Automatically calls ovphysx_destroy_instance on destruction.
 * Move-only (non-copyable) to ensure unique ownership.
 *
 * Provides implicit conversion to ovphysx_instance_handle_t for use with the C API.
 *
 * Example:
 *   ovphysx_initialize();
 *   {
 *       CreateArgs args;
 *       PhysX physx;
 *       PhysX::create(physx, args);
 *       physx.step(0.01f);
 *       physx.waitAll();
 *   }
 *   ovphysx_shutdown();
 *
 * Notes:
 *   - Use PhysX::create to obtain a valid instance. Methods log and return errors if the handle is null.
 *   - Use waitTask/waitAll when results are needed outside stream order.
 */
class OVPHYSX_API PhysX {
public:
    /// Construct from existing handle (takes ownership)
    explicit PhysX(ovphysx_instance_handle_t h);
    
    /// Default constructor - creates null handle
    PhysX();
    
    /// Destroy a valid instance. Close leases and finish concurrent operations first.
    /// A retryable rejection terminates; use reset() to handle and retry it before destruction.
    ~PhysX();
    
    /// Move constructor
    PhysX(PhysX&& other) noexcept;
    
    /// Move assignment swaps handles; the source owns the destination's previous handle.
    PhysX& operator=(PhysX&& other) noexcept;
    
    // Non-copyable
    PhysX(const PhysX&) = delete;
    PhysX& operator=(const PhysX&) = delete;
    
    /// Get raw handle
    ovphysx_instance_handle_t handle() const { return m_handle; }
    
    /// Implicit conversion to handle for use with C API
    operator ovphysx_instance_handle_t() const { return m_handle; }
    
    /// Check if handle is valid
    explicit operator bool() const { return m_handle != 0; }
    
    /// Release ownership of handle (caller must destroy)
    ovphysx_instance_handle_t release();
    
    /// Destroy the current owner then adopt h. A retryable rejection throws
    /// std::runtime_error and preserves both handles; h remains caller-owned.
    void reset(ovphysx_instance_handle_t h = 0);
    
    //------------------------------------------------------------------------------------------------------------
    // Stage Management
    //------------------------------------------------------------------------------------------------------------

    /// Detach the Stage without changing simulation time; already detached is a successful no-op.
    /// Executes inline and returns a task receipt; waitTask() or waitAll() collects its outcome.
    ovphysx_enqueue_result_t detachOvstage();

    /// Attach an ovstage Stage through the top-level ovphysx API. `read_ordinal`
    /// selects the caller-owned sealed ordinal (must be non-zero, 0 is reserved).
    /// This instance may hold one attach. Other instances may be attached at the
    /// same time; this call does not detach them.
    /// Returns a task receipt; waitTask() or waitAll() collects its outcome.
    ovphysx_enqueue_result_t attachOvstage(ovstage_instance_t* stage, ovstage_ordinal_t read_ordinal);

    /// Pull and apply ovstage changes over the committed ordinal range.
    /// Returns a task receipt; waitTask() or waitAll() collects its outcome.
    ovphysx_enqueue_result_t updateFromOvstage(ovstage_ordinal_range_t range);

    /// Clone a USD prim hierarchy to create multiple runtime physics copies.
    ///
    /// Creates physics-optimized clones in the internal representation for
    /// high-performance simulation. The source prim must exist in the loaded
    /// USD stage and have physics properties. Replication runs inline (any returned
    /// op index is already complete), backed by the PhysX SDK replicator, so cloned
    /// articulations are real articulations.
    ///
    /// This is the clone entrypoint for both standalone callers and callers
    /// that populate the scene through an ovstage Stage attached via the C API
    /// `ovphysx_attach_ovstage`. Replication runs in the internal representation
    /// only (USD untouched).
    ///
    /// @param sourcePath USD path of the source prim hierarchy (e.g., "/World/env0")
    /// @param targetPaths Vector of runtime physics-object paths for cloned hierarchies
    ///        (e.g., ["/World/env1", "/World/env2"])
    /// @param anchorTransforms Absolute world pose of each target subtree root. Entry i anchors
    ///        the exact subtree at targetPaths[i]. Flat array of [targetPaths.size() * 7] floats:
    ///        (px, py, pz, qx, qy, qz, qw) per target. Descendants keep their poses relative to
    ///        the source subtree root (targetObjectWorld = anchorTransforms[i] *
    ///        inverse(sourceRootWorld) * sourceObjectWorld). Pass nullptr to co-locate every
    ///        copy on the source.
    /// @param envIds Optional logical environment id per target ([targetPaths.size()]
    ///        uint32, each < 0x00FFFFFF, runtime id = envIds[i]+1). Stable across calls:
    ///        the same id maps to the same environment, so clones sharing an id collide
    ///        and stay isolated from other environments. Pass nullptr for automatic
    ///        per-call numbering.
    /// @return Admission receipt; collect execution errors with waitTask() or waitAll().
    ovphysx_enqueue_result_t clone(const std::string& sourcePath, const std::vector<std::string>& targetPaths,
                               const float* anchorTransforms = nullptr,
                               const uint32_t* envIds = nullptr);


    //------------------------------------------------------------------------------------------------------------
    // Simulation
    //------------------------------------------------------------------------------------------------------------
    
    /// Enqueue a physics simulation step (simulation time tracked internally).
    ovphysx_enqueue_result_t step(float step_dt);

    /// Recompute selected articulation link state; collect the task's outcome with waitTask().
    ovphysx_enqueue_result_t updateArticulationsKinematic(ovphysx_query_handle_t query, uint32_t flags);

    /// Evaluate the SDFs of an SDF_SHAPE query's shapes at the given points; collect the outcome with waitTask().
    /// `pointsReady` is the producer handoff for `points`, as for ovphysx_evaluate_sdf_shapes.
    ovphysx_enqueue_result_t evaluateSdfShapes(ovphysx_query_handle_t query,
                                               const DLTensor* points,
                                               DLTensor* out,
                                               ovstage_cuda_sync_t pointsReady = {});
    
    //------------------------------------------------------------------------------------------------------------
    // Synchronization
    //------------------------------------------------------------------------------------------------------------
    
    /// Collect only this task; inspect the returned status, errors and pending task.
    physx::WaitResult waitTask(
        ovphysx_enqueue_result_t task,
        ovphysx_timeout_t timeout_ns = OVPHYSX_TIMEOUT_INFINITE);
    
    /// Collect all uncollected tasks accepted before this call.
    physx::WaitResult waitAll(ovphysx_timeout_t timeout_ns = OVPHYSX_TIMEOUT_INFINITE);

    //------------------------------------------------------------------------------------------------------------
    // Tensor Bindings
    //------------------------------------------------------------------------------------------------------------

    /**
     * @brief Create a tensor binding for bulk physics data access
     *
     * Creates a binding that connects physics-object paths (matched by pattern) to a tensor type,
     * enabling efficient bulk read/write for authored USD objects and runtime-only clones.
     *
     * @param out_binding  Receives the created TensorBinding on success
     * @param pattern      Physics-object path pattern (e.g., "/World/robot*")
     * @param tensor_type  The type of tensor data to bind
     * @return OVPHYSX_API_SUCCESS on success
     *
     * @deprecated The tensor-binding API is deprecated. Use @ref ovphysx_read
     *   (reads) and @ref ovphysx_write (writes) instead.
     */
    OVPHYSX_DEPRECATED_MSG(
        "ovphysx tensor bindings are deprecated; use ovphysx_read (reads) / ovphysx_write (writes)")
    ovphysx_api_status_t createTensorBinding(
        TensorBinding& out_binding,
        const std::string& pattern,
        ovphysx_tensor_type_t tensor_type);

    //------------------------------------------------------------------------------------------------------------
    // Scene queries
    //------------------------------------------------------------------------------------------------------------

    using SceneQueryHit = ovphysx_scene_query_hit_t;

    /**
     * @brief Cast a ray into the scene.
     *
     * @param origin     Ray origin (world space).
     * @param direction  Normalized ray direction.
     * @param distance   Maximum ray length.
     * @param both_sides Test both sides of mesh triangles.
     * @param mode       CLOSEST, ANY, or ALL.
     * @param[out] hits  Pointer to internal hit array (valid until next scene query call).
     * @param[out] count Number of hits.
     */
    OVPHYSX_DEPRECATED_MSG("ovphysx scene queries are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_api_status_t raycast(
        const float origin[3], const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const SceneQueryHit*& hits, uint32_t& count) const
    {
        OVPHYSX_DEPRECATED_CALL_BEGIN
        ovphysx_result_t r = ovphysx_raycast(
            m_handle, origin, direction, distance, both_sides, mode, &hits, &count);
        OVPHYSX_DEPRECATED_CALL_END
        return r.status;
    }

    /**
     * @brief Sweep a geometry shape through the scene.
     *
     * @param geometry   Geometry descriptor.
     * @param direction  Normalized sweep direction.
     * @param distance   Maximum sweep length.
     * @param both_sides Test both sides of mesh triangles.
     * @param mode       CLOSEST, ANY, or ALL.
     * @param[out] hits  Pointer to internal hit array (valid until next scene query call).
     * @param[out] count Number of hits.
     */
    OVPHYSX_DEPRECATED_MSG("ovphysx scene queries are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_api_status_t sweep(
        const ovphysx_scene_query_geometry_desc_t& geometry,
        const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const SceneQueryHit*& hits, uint32_t& count) const
    {
        OVPHYSX_DEPRECATED_CALL_BEGIN
        ovphysx_result_t r = ovphysx_sweep(
            m_handle, &geometry, direction, distance, both_sides, mode, &hits, &count);
        OVPHYSX_DEPRECATED_CALL_END
        return r.status;
    }

    /**
     * @brief Test geometry overlap against objects in the scene.
     *
     * @param geometry   Geometry descriptor.
     * @param mode       ANY or ALL. CLOSEST falls back to ALL because overlap
     *                   tests have no distance ordering.
     * @param[out] hits  Pointer to internal hit array (valid until next scene query call).
     * @param[out] count Number of overlapping objects.
     */
    OVPHYSX_DEPRECATED_MSG("ovphysx scene queries are deprecated; include <ovphysx/ovphysx_legacy.h>")
    ovphysx_api_status_t overlap(
        const ovphysx_scene_query_geometry_desc_t& geometry,
        ovphysx_scene_query_mode_t mode,
        const SceneQueryHit*& hits, uint32_t& count) const
    {
        OVPHYSX_DEPRECATED_CALL_BEGIN
        ovphysx_result_t r = ovphysx_overlap(m_handle, &geometry, mode, &hits, &count);
        OVPHYSX_DEPRECATED_CALL_END
        return r.status;
    }

    //------------------------------------------------------------------------------------------------------------
    // Factory
    //------------------------------------------------------------------------------------------------------------
    
    /** Create an instance with default instance configuration. */
    static ovphysx_api_status_t create(PhysX& out_instance);

    /** @deprecated Set global config before create and instance config after create. */
    OVPHYSX_DEPRECATED_MSG("use create(out_instance), then setConfig")
    static ovphysx_api_status_t create(
        PhysX& out_instance,
        const CreateArgs& args);

    ovphysx_api_status_t setConfig(ovphysx_config_key_t key, ovphysx_config_value_t value)
    {
        return ovphysx_set_config(m_handle, key, value).status;
    }

    /**
     * Get one instance value. For string keys, initialize value as
     * OVPHYSX_CONFIG_VALUE_STRING with caller-owned storage. A zero-capacity
     * query returns OVPHYSX_API_BUFFER_TOO_SMALL and the required capacity.
     */
    ovphysx_api_status_t getConfig(ovphysx_config_key_t key, ovphysx_config_value_t& value) const
    {
        return ovphysx_get_config(m_handle, key, &value).status;
    }

    /// @deprecated Use setGlobalConfig(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, ...).
    OVPHYSX_DEPRECATED_MSG("use setGlobalConfig")
    static ovphysx_api_status_t setCpuMode(bool cpuOnly);

    /// @deprecated Use getGlobalConfig(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, ...).
    OVPHYSX_DEPRECATED_MSG("use getGlobalConfig")
    static ovphysx_api_status_t getCpuMode(bool& outCpuOnly);

private:
    ovphysx_instance_handle_t m_handle;
};

inline ovphysx_api_status_t setGlobalConfig(
    ovphysx_global_config_key_t key,
    ovphysx_config_value_t value)
{
    return ovphysx_set_global_config(key, value).status;
}

/** String keys use the same caller-owned buffer protocol as PhysX::getConfig(). */
inline ovphysx_api_status_t getGlobalConfig(
    ovphysx_global_config_key_t key,
    ovphysx_config_value_t& value)
{
    return ovphysx_get_global_config(key, &value).status;
}

/**
 * @brief Clear the ovphysx process-lifecycle token.
 *
 * Thin wrapper for @ref ovphysx_shutdown(). See the C header for full
 * semantics. This does not destroy live handles. With no live handles it
 * drains the direct PhysX runtime. Linux also releases the SDK-owned Carbonite
 * framework and private cache; Windows retains its framework until process exit
 * and private-cache removal remains best-effort.
 *
 * @return OVPHYSX_API_SUCCESS on success, OVPHYSX_API_ERROR if called without
 *         a matching ovphysx_initialize().
 */
inline ovphysx_api_status_t shutdown()
{
    return ovphysx_shutdown().status;
}

// ============================================================================
// Object change notifications (per instance)
// ============================================================================

/**
 * @brief Callback set for ObjectChangeSubscription.
 *
 * Each callback is optional. An unset std::function is skipped rather than
 * called. At least one of the three must be set, otherwise
 * subscribeObjectChanges() returns an inactive ObjectChangeSubscription
 * (check ObjectChangeSubscription::isActive() to detect this).
 *
 * Threading: callbacks may fire from internal worker threads during
 * PhysX::step() or PhysX::detachOvstage(). Do not call other ovphysx APIs from
 * inside a callback. Defer that work to after the next waitTask() /
 * waitAll() returns.
 *
 * PhysX::clone() does NOT emit onCreated for clone-replicated objects.
 * Refresh cached PhysX pointers after PhysX::clone() returns and
 * PhysX::waitAll() completes, not by waiting for a callback.
 */
struct ObjectChangeCallbacks {
    /// Fires AFTER the object is created. Safe to read physxPointer
    /// from a deferred handler.
    std::function<void(std::string_view primPath, ovphysx_physx_type_t type)> onCreated;

    /// Fires BEFORE the object is destroyed. Drop any cached pointer for
    /// primPath / type before returning. It is unusable once this callback returns.
    std::function<void(std::string_view primPath, ovphysx_physx_type_t type)> onDestroyed;

    /// Fires BEFORE a bulk teardown (e.g. PhysX::detachOvstage()). Flush the entire
    /// pointer cache before returning. No per-object onDestroyed events follow.
    std::function<void()> onAllDestroyed;
};

/** @cond */
namespace detail {
/// Close a subscription without waiting. Used when a callback is active on
/// this Carbonite task, or on this native thread when the callback is not a
/// Carbonite task, so ovphysx_unsubscribe_object_changes would return
/// INVALID_STATE. Returns 1 when the caller must delete user_data now, 0 when
/// user_data is freed after the running callback has unwound.
OVPHYSX_API int closeObjectChangeFromCallback(
    ovphysx_instance_handle_t handle,
    ovphysx_subscription_id_t subscription,
    void (*releaseUserData)(void*));
void releaseObjectChangeState(void* userData);
} // namespace detail
/** @endcond */

/**
 * @brief RAII subscription handle returned by subscribeObjectChanges().
 *
 * On destruction, unsubscribes via ovphysx_unsubscribe_object_changes() so the
 * callbacks stop firing. Movable, non-copyable.
 */
class ObjectChangeSubscription {
public:
    ObjectChangeSubscription() noexcept : m_handle(OVPHYSX_INVALID_HANDLE), m_id(OVPHYSX_INVALID_SUBSCRIPTION_ID) {}

    ObjectChangeSubscription(const ObjectChangeSubscription&) = delete;
    ObjectChangeSubscription& operator=(const ObjectChangeSubscription&) = delete;

    ObjectChangeSubscription(ObjectChangeSubscription&& other) noexcept
        : m_handle(other.m_handle), m_id(other.m_id), m_state(std::move(other.m_state))
    {
        other.m_handle = OVPHYSX_INVALID_HANDLE;
        other.m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    }

    ObjectChangeSubscription& operator=(ObjectChangeSubscription&& other) noexcept
    {
        if (this != &other) {
            releaseHandle();
            m_handle = other.m_handle;
            m_id = other.m_id;
            m_state = std::move(other.m_state);
            other.m_handle = OVPHYSX_INVALID_HANDLE;
            other.m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
        }
        return *this;
    }

    ~ObjectChangeSubscription()
    {
        releaseHandle();
    }

    /// Returns true if this handle owns a live subscription.
    bool isActive() const noexcept { return m_id != OVPHYSX_INVALID_SUBSCRIPTION_ID; }

    /// Underlying C subscription ID (or OVPHYSX_INVALID_SUBSCRIPTION_ID if inactive).
    ovphysx_subscription_id_t id() const noexcept { return m_id; }

    /// Unsubscribe explicitly. Idempotent after a successful removal.
    ///
    /// On success, or when instance teardown already removed the subscription,
    /// both m_id and m_state are cleared. user_data is safe to drop only after
    /// that success: the C unsubscribe waits until this subscription's callbacks
    /// have returned.
    ///
    /// OVPHYSX_API_INVALID_STATE means an object-change callback is running on
    /// this Carbonite task, or on this native thread when the callback is not
    /// a Carbonite task. The subscription stays active. Retry after that
    /// callback returns. Destroying or move-assigning the handle in that
    /// situation closes it without waiting: later callbacks are skipped, and
    /// the callback state is freed once that call has unwound, including when
    /// the callback throws.
    ///
    /// For any other C unsubscribe failure (e.g. internal sidecar unloaded),
    /// m_state is intentionally leaked rather than freed. The C-side subscription
    /// may still hold a pointer to it. m_id is still cleared so the handle is
    /// considered consumed by the caller.
    ovphysx_api_status_t unsubscribe()
    {
        if (m_id == OVPHYSX_INVALID_SUBSCRIPTION_ID) {
            // Already inactive. Nothing to free (no C-side subscription).
            return OVPHYSX_API_SUCCESS;
        }
        ovphysx_result_t r = ovphysx_unsubscribe_object_changes(m_handle, m_id);
        if (r.status == OVPHYSX_API_INVALID_STATE) {
            return r.status;
        }
        m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
        if (r.status == OVPHYSX_API_SUCCESS || r.status == OVPHYSX_API_NOT_FOUND ||
            r.status == OVPHYSX_API_INVALID_ARGUMENT) {
            m_state.reset();
        } else {
            // Leak: keep m_state alive so any pending C-side callback finds
            // a valid pointer. The leak is permanent for this subscription.
            (void)m_state.release();
        }
        return r.status;
    }

    // State is the heap-allocated owner of the std::function callbacks. The
    // C trampolines below receive a State* via the C user_data pointer and
    // dispatch through it. Public so namespace-scope trampolines can reach the
    // member. Users should not touch this struct directly.
    struct State {
        ObjectChangeCallbacks callbacks;
    };

private:
    friend ObjectChangeSubscription subscribeObjectChanges(ovphysx_instance_handle_t, ObjectChangeCallbacks);

    void releaseHandle() noexcept
    {
        if (m_id == OVPHYSX_INVALID_SUBSCRIPTION_ID)
            return;
        const ovphysx_result_t result = ovphysx_unsubscribe_object_changes(m_handle, m_id);
        if (result.status == OVPHYSX_API_INVALID_STATE) {
            const int releaseNow = detail::closeObjectChangeFromCallback(
                m_handle, m_id, &detail::releaseObjectChangeState);
            m_handle = OVPHYSX_INVALID_HANDLE;
            m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
            if (releaseNow)
                m_state.reset();
            else
                (void)m_state.release();
            return;
        }
        m_handle = OVPHYSX_INVALID_HANDLE;
        m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
        if (result.status == OVPHYSX_API_SUCCESS || result.status == OVPHYSX_API_NOT_FOUND ||
            result.status == OVPHYSX_API_INVALID_ARGUMENT) {
            m_state.reset();
        } else {
            (void)m_state.release();
        }
    }

    ovphysx_instance_handle_t m_handle = OVPHYSX_INVALID_HANDLE;
    ovphysx_subscription_id_t m_id;
    std::unique_ptr<State> m_state;
};

namespace detail {
/** @cond */
inline void releaseObjectChangeState(void* userData)
{
    delete static_cast<ObjectChangeSubscription::State*>(userData);
}
/** @endcond */

inline void objectChangeCreatedTrampoline(ovphysx_string_t primPath, ovphysx_physx_type_t type, void* userData)
{
    auto* state = static_cast<ObjectChangeSubscription::State*>(userData);
    if (state && state->callbacks.onCreated) {
        state->callbacks.onCreated(std::string_view(primPath.ptr, primPath.length), type);
    }
}

inline void objectChangeDestroyedTrampoline(ovphysx_string_t primPath, ovphysx_physx_type_t type, void* userData)
{
    auto* state = static_cast<ObjectChangeSubscription::State*>(userData);
    if (state && state->callbacks.onDestroyed) {
        state->callbacks.onDestroyed(std::string_view(primPath.ptr, primPath.length), type);
    }
}

inline void objectChangeAllDestroyedTrampoline(void* userData)
{
    auto* state = static_cast<ObjectChangeSubscription::State*>(userData);
    if (state && state->callbacks.onAllDestroyed) {
        state->callbacks.onAllDestroyed();
    }
}
} // namespace detail

/**
 * @brief Subscribe to PhysX object create/destroy notifications.
 *
 * Returns an RAII handle whose destructor calls
 * ovphysx_unsubscribe_object_changes(). On failure (no callbacks set, internal
 * sidecar not loaded, etc.) the returned handle satisfies
 * `!handle.isActive()`. The underlying C call's error is not exposed through
 * this overload. Subscriptions are per-instance: pass the owning handle.
 * See the docstring on ovphysx_subscribe_object_changes in ovphysx.h for the
 * full contract.
 */
inline ObjectChangeSubscription subscribeObjectChanges(
    ovphysx_instance_handle_t handle, ObjectChangeCallbacks callbacks)
{
    ObjectChangeSubscription sub;
    sub.m_state = std::make_unique<ObjectChangeSubscription::State>();
    sub.m_state->callbacks = std::move(callbacks);

    ovphysx_object_change_callbacks_t c{};
    c.on_object_created        = sub.m_state->callbacks.onCreated       ? detail::objectChangeCreatedTrampoline       : nullptr;
    c.on_object_destroyed      = sub.m_state->callbacks.onDestroyed     ? detail::objectChangeDestroyedTrampoline     : nullptr;
    c.on_all_objects_destroyed = sub.m_state->callbacks.onAllDestroyed  ? detail::objectChangeAllDestroyedTrampoline  : nullptr;
    c.user_data                = sub.m_state.get();

    ovphysx_subscription_id_t id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    if (ovphysx_subscribe_object_changes(handle, &c, &id).status != OVPHYSX_API_SUCCESS) {
        sub.m_state.reset();
        return sub;  // m_id stays OVPHYSX_INVALID_SUBSCRIPTION_ID
    }
    sub.m_handle = handle;
    sub.m_id = id;
    return sub;
}

} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_OVPHYSX_HPP
