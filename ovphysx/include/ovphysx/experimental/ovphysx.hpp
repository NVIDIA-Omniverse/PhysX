// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef OVPHYSX_EXPERIMENTAL_OVPHYSX_HPP
#define OVPHYSX_EXPERIMENTAL_OVPHYSX_HPP

//
// C++17 RAII wrapper for ovphysx instance handle
// Provides automatic cleanup and thin C++ wrappers for all C API functions
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

// Forward declarations of PhysX SDK types used by the type-safe getPhysXPtr<T>()
// overload below. This avoids requiring consumers to include the full PhysX SDK
// headers unless they actually call getPhysXPtr. The PhysX SDK headers can be
// obtained from https://github.com/NVIDIA-Omniverse/PhysX -- use the version
// matching the ovphysx release (currently PhysX 5.x).
namespace physx {
class PxScene;
class PxMaterial;
class PxShape;
class PxRigidActor;
class PxJoint;
class PxArticulationReducedCoordinate;
class PxArticulationLink;
class PxArticulationJointReducedCoordinate;
} // namespace physx

namespace ovphysx {

/**
 * @brief Safe wrapper for ovphysx_create_args.
 *
 * Default-constructs to OVPHYSX_CREATE_ARGS_DEFAULT (empty active_cuda_gpus,
 * no config entries, empty bundled deps path). Use setters to override individual
 * fields before passing to PhysX::create().
 *
 * Callers do not need to touch ovphysx_create_args directly — this class
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
    ///              CreateArgs copies the string internally; the caller does not need
    ///              to keep the argument alive after this call returns.
    void setActiveCudaGpus(const std::string& gpus);
    /// @param path  Bundled deps path.
    ///              CreateArgs copies the string internally; the caller does not need
    ///              to keep the argument alive after this call returns.
    void setBundledDepsPath(const std::string& path);

    /// @param entries  Pointer to an array of config entries. The caller must keep this array
    ///                 valid until PhysX::create() returns — CreateArgs does not copy the data.
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

/// Traits mapping a PhysX SDK type to its ovphysx_physx_type_t enum value.
/// Enables type-safe getPhysXPtr() overloads that auto-deduce the enum.
template <typename T> struct PhysXTypeFor;
template <> struct PhysXTypeFor<::physx::PxScene>                              { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_SCENE; };
template <> struct PhysXTypeFor<::physx::PxMaterial>                           { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_MATERIAL; };
template <> struct PhysXTypeFor<::physx::PxShape>                              { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_SHAPE; };
template <> struct PhysXTypeFor<::physx::PxRigidActor>                         { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_ACTOR; };
template <> struct PhysXTypeFor<::physx::PxJoint>                              { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_JOINT; };
template <> struct PhysXTypeFor<::physx::PxArticulationReducedCoordinate>      { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_ARTICULATION; };
template <> struct PhysXTypeFor<::physx::PxArticulationLink>                   { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_LINK; };
template <> struct PhysXTypeFor<::physx::PxArticulationJointReducedCoordinate> { static constexpr ovphysx_physx_type_t value = OVPHYSX_PHYSX_TYPE_LINK_JOINT; };

/**
 * @brief RAII wrapper for ovphysx_handle_t
 * 
 * Automatically calls ovphysx_destroy_instance on destruction.
 * Move-only (non-copyable) to ensure unique ownership.
 * 
 * Provides implicit conversion to ovphysx_handle_t for seamless use with C API.
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
 *   - Use PhysX::create to obtain a valid instance; methods log and return errors if the handle is null.
 *   - Use waitOp/waitAll when you need results outside stream order.
 */
class OVPHYSX_API PhysX {
public:
    /// Construct from existing handle (takes ownership)
    explicit PhysX(ovphysx_handle_t h);
    
    /// Default constructor - creates null handle
    PhysX();
    
    /// Destructor - destroys instance if valid
    ~PhysX();
    
    /// Move constructor
    PhysX(PhysX&& other) noexcept;
    
    /// Move assignment
    PhysX& operator=(PhysX&& other) noexcept;
    
    // Non-copyable
    PhysX(const PhysX&) = delete;
    PhysX& operator=(const PhysX&) = delete;
    
    /// Get raw handle
    ovphysx_handle_t handle() const { return m_handle; }
    
    /// Implicit conversion to handle for use with C API
    operator ovphysx_handle_t() const { return m_handle; }
    
    /// Check if handle is valid
    explicit operator bool() const { return m_handle != 0; }
    
    /// Release ownership of handle (caller must destroy)
    ovphysx_handle_t release();
    
    /// Reset to new handle (destroys current if valid)
    void reset(ovphysx_handle_t h = 0);
    
    //------------------------------------------------------------------------------------------------------------
    // Stage Management
    //------------------------------------------------------------------------------------------------------------

    /// Reset the stage to empty (does not change the simulation-time counter)
    ovphysx_api_status_t reset_stage();

    /// Attach an ovstage Stage through the top-level ovphysx API. `read_ordinal`
    /// is the caller-owned sealed ordinal the initial scene parse reads at.
    ovphysx_api_status_t attachOvstage(ovstage_instance_t* stage, ovstage_ordinal_t read_ordinal);

    /// Pull and apply ovstage changes over the committed ordinal range.
    ovphysx_api_status_t updateFromOvstage(ovstage_ordinal_range_t range);

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
    /// @param parentTransforms World pose of each copy's parent. Flat array of
    ///        [targetPaths.size() * 7] floats: (px, py, pz, qx, qy, qz, qw) per target
    ///        (copy = transform * inverse(source_parent) * body). Pass nullptr to
    ///        co-locate every copy on the source.
    /// @param envIds Optional logical environment id per target ([targetPaths.size()]
    ///        uint32, each < 0x00FFFFFF; runtime id = envIds[i]+1). Stable across calls:
    ///        the same id maps to the same environment, so clones sharing an id collide
    ///        and stay isolated from other environments. Pass nullptr for automatic
    ///        per-call numbering.
    /// @param outOpIndex Optional; receives the clone operation index on success
    ///        (usable with waitOp()). The clone completes synchronously, so waiting is
    ///        only for API uniformity.
    /// @return OVPHYSX_API_SUCCESS if cloning succeeded, OVPHYSX_API_ERROR on error
    ovphysx_api_status_t clone(const std::string& sourcePath, const std::vector<std::string>& targetPaths,
                               const float* parentTransforms = nullptr,
                               const uint32_t* envIds = nullptr,
                               ovphysx_op_index_t* outOpIndex = nullptr);


    //------------------------------------------------------------------------------------------------------------
    // Simulation
    //------------------------------------------------------------------------------------------------------------
    
    /// Enqueue a physics simulation step (simulation time tracked internally).
    ovphysx_api_status_t step(float step_dt);

    /// Recompute articulation link poses from current joint positions without stepping simulation.
    ovphysx_api_status_t updateArticulationsKinematic();
    
    //------------------------------------------------------------------------------------------------------------
    // User Tasks
    //------------------------------------------------------------------------------------------------------------
    
    /// Add a user task to the execution queue
    ovphysx_api_status_t addUserTask(const ovphysx_user_task_desc_t& desc,
                                      ovphysx_op_index_t& out_op_index);
    
    //------------------------------------------------------------------------------------------------------------
    // Synchronization
    //------------------------------------------------------------------------------------------------------------
    
    /// Wait for a specific operation to complete
    physx::WaitResult waitOp(ovphysx_op_index_t op_index, uint64_t timeout_ns = UINT64_MAX);
    
    /// Wait for all pending operations to complete
    physx::WaitResult waitAll(uint64_t timeout_ns = UINT64_MAX);

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
     */
    ovphysx_api_status_t createTensorBinding(
        TensorBinding& out_binding,
        const std::string& pattern,
        ovphysx_tensor_type_t tensor_type);

    //------------------------------------------------------------------------------------------------------------
    // PhysX object interop
    //------------------------------------------------------------------------------------------------------------

    /**
     * @brief Type-safe accessor: deduces the enum from the PhysX pointer type.
     *
     * Example: `physx::PxScene* s; physx.getPhysXPtr("/World/scene", s);`
     * Compile error if T has no PhysXTypeFor<T> specialization.
     */
    template <typename T>
    ovphysx_api_status_t getPhysXPtr(const std::string& primPath,
                                     T*& out) const {
        void* raw = nullptr;
        auto r = ovphysx_get_physx_ptr(m_handle, {primPath.c_str(), primPath.size()},
                                       PhysXTypeFor<T>::value, &raw);
        out = static_cast<T*>(raw);
        return r.status;
    }

    /**
     * @brief Explicit-enum accessor for advanced use or unsupported types.
     *
     * Prefer the two-argument overload above when T is a known PhysX type.
     */
    ovphysx_api_status_t getPhysXPtr(const std::string& primPath,
                                     ovphysx_physx_type_t type,
                                     void*& out) const {
        auto r = ovphysx_get_physx_ptr(m_handle, {primPath.c_str(), primPath.size()}, type, &out);
        return r.status;
    }

    //------------------------------------------------------------------------------------------------------------
    // Contact report
    //------------------------------------------------------------------------------------------------------------

    using ContactEventHeader = ovphysx_contact_event_header_t;
    using ContactPoint = ovphysx_contact_point_t;
    using FrictionAnchor = ovphysx_friction_anchor_t;

    /**
     * @brief Get contact report data for the current simulation step.
     *
     * Returns typed pointers to the internal contact buffers.
     * Data is valid until the next simulation step.
     *
     * @param[out] headers   Pointer to contact event header array.
     * @param[out] numHeaders Number of headers.
     * @param[out] points    Pointer to contact point data array.
     * @param[out] numPoints Number of contact point entries.
     * @param[out] anchors   Optional. Pointer to friction anchor array (pass nullptr to skip).
     * @param[out] numAnchors Optional. Friction anchor count (pass nullptr to skip).
     */
    ovphysx_api_status_t getContactReport(
        const ContactEventHeader*& headers, uint32_t& numHeaders,
        const ContactPoint*& points, uint32_t& numPoints,
        const FrictionAnchor** anchors = nullptr, uint32_t* numAnchors = nullptr) const
    {
        auto r = ovphysx_get_contact_report(
            m_handle, &headers, &numHeaders, &points, &numPoints,
            anchors, numAnchors);
        return r.status;
    }

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
    ovphysx_api_status_t raycast(
        const float origin[3], const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const SceneQueryHit*& hits, uint32_t& count) const
    {
        auto r = ovphysx_raycast(m_handle, origin, direction, distance, both_sides, mode, &hits, &count);
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
    ovphysx_api_status_t sweep(
        const ovphysx_scene_query_geometry_desc_t& geometry,
        const float direction[3],
        float distance,
        bool both_sides,
        ovphysx_scene_query_mode_t mode,
        const SceneQueryHit*& hits, uint32_t& count) const
    {
        auto r = ovphysx_sweep(m_handle, &geometry, direction, distance, both_sides, mode, &hits, &count);
        return r.status;
    }

    /**
     * @brief Test geometry overlap against objects in the scene.
     *
     * @param geometry   Geometry descriptor.
     * @param mode       ANY or ALL.  CLOSEST falls back to ALL because overlap
     *                   tests have no distance ordering.
     * @param[out] hits  Pointer to internal hit array (valid until next scene query call).
     * @param[out] count Number of overlapping objects.
     */
    ovphysx_api_status_t overlap(
        const ovphysx_scene_query_geometry_desc_t& geometry,
        ovphysx_scene_query_mode_t mode,
        const SceneQueryHit*& hits, uint32_t& count) const
    {
        auto r = ovphysx_overlap(m_handle, &geometry, mode, &hits, &count);
        return r.status;
    }

    //------------------------------------------------------------------------------------------------------------
    // Factory
    //------------------------------------------------------------------------------------------------------------
    
    /**
     * @brief Factory method to create a PhysX instance from CreateArgs.
     *
     * This is the primary creation path. Use CreateArgs to configure device
     * selection, GPU index, config entries, and other options.
     *
     * Initialize the C API first, construct CreateArgs, call create(), and check
     * its returned status. Destroy the PhysX instance before calling
     * ovphysx_shutdown().
     *
     * @param out_instance Receives the created PhysX instance on success.
     * @param args         Creation arguments (default-constructed = OVPHYSX_CREATE_ARGS_DEFAULT).
     * @return OVPHYSX_API_SUCCESS on success; OVPHYSX_API_INVALID_ARGUMENT on
     *         inconsistent args (e.g. config_entry_count > 0 with null pointer).
     */
    static ovphysx_api_status_t create(
        PhysX& out_instance,
        const CreateArgs& args);

    /// @brief Force process-wide CPU-only mode. Must be called before any instances are active.
    ///        See ovphysx_set_cpu_mode() for full semantics.
    static ovphysx_api_status_t setCpuMode(bool cpuOnly);

private:
    ovphysx_handle_t m_handle;
};

/**
 * @brief Clear the ovphysx process-lifecycle token.
 *
 * Thin wrapper for @ref ovphysx_shutdown(). See the C header for full
 * semantics. This does not destroy live handles; Carbonite and the static
 * PhysX runtime remain resident until process exit.
 *
 * @return OVPHYSX_API_SUCCESS on success, OVPHYSX_API_ERROR if called without
 *         a matching ovphysx_initialize().
 */
inline ovphysx_api_status_t shutdown()
{
    return ovphysx_shutdown().status;
}

// ============================================================================
// Object change notifications (process-global)
// ============================================================================

/**
 * @brief Callback set for ObjectChangeSubscription.
 *
 * Each callback is optional; an unset std::function is skipped rather than
 * called. At least one of the three must be set, otherwise
 * subscribeObjectChanges() returns an inactive ObjectChangeSubscription
 * (check ObjectChangeSubscription::isActive() to detect this).
 *
 * Threading: callbacks may fire from internal worker threads during
 * PhysX::step() or PhysX::reset_stage(). Do not call other ovphysx APIs from
 * inside a callback -- defer that work to after the next waitOp() /
 * waitAll() returns.
 *
 * PhysX::clone() does NOT emit onCreated for clone-replicated objects.
 * Refresh cached PhysX pointers after PhysX::clone() returns and
 * PhysX::waitAll() completes, not by waiting for a callback.
 */
struct ObjectChangeCallbacks {
    /// Fires AFTER the object is created. Safe to call PhysX::getPhysXPtr()
    /// from a deferred handler.
    std::function<void(std::string_view primPath, ovphysx_physx_type_t type)> onCreated;

    /// Fires BEFORE the object is destroyed. Drop any cached pointer for
    /// primPath / type at this point.
    std::function<void(std::string_view primPath, ovphysx_physx_type_t type)> onDestroyed;

    /// Fires BEFORE a bulk teardown (e.g. PhysX::reset_stage()). Flush the entire
    /// pointer cache; no per-object onDestroyed events follow.
    std::function<void()> onAllDestroyed;
};

/**
 * @brief RAII subscription handle returned by subscribeObjectChanges().
 *
 * On destruction, unsubscribes via ovphysx_unsubscribe_object_changes() so the
 * callbacks stop firing. Movable, non-copyable.
 */
class ObjectChangeSubscription {
public:
    ObjectChangeSubscription() noexcept : m_id(OVPHYSX_INVALID_SUBSCRIPTION_ID) {}

    ObjectChangeSubscription(const ObjectChangeSubscription&) = delete;
    ObjectChangeSubscription& operator=(const ObjectChangeSubscription&) = delete;

    ObjectChangeSubscription(ObjectChangeSubscription&& other) noexcept
        : m_id(other.m_id), m_state(std::move(other.m_state))
    {
        other.m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
    }

    ObjectChangeSubscription& operator=(ObjectChangeSubscription&& other) noexcept
    {
        if (this != &other) {
            unsubscribe();
            m_id = other.m_id;
            m_state = std::move(other.m_state);
            other.m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
        }
        return *this;
    }

    ~ObjectChangeSubscription() { unsubscribe(); }

    /// Returns true if this handle owns a live subscription.
    bool isActive() const noexcept { return m_id != OVPHYSX_INVALID_SUBSCRIPTION_ID; }

    /// Underlying C subscription ID (or OVPHYSX_INVALID_SUBSCRIPTION_ID if inactive).
    ovphysx_subscription_id_t id() const noexcept { return m_id; }

    /// Unsubscribe explicitly. Idempotent.
    ///
    /// On success, both m_id and m_state are cleared. If the underlying C
    /// unsubscribe FAILS (e.g. internal sidecar unloaded), m_state is intentionally
    /// leaked rather than freed -- the C-side subscription may still hold a
    /// pointer to it, and freeing would leave a dangling user_data for any
    /// in-flight or queued callback. m_id is still cleared so the handle is
    /// considered consumed by the caller.
    void unsubscribe()
    {
        if (m_id == OVPHYSX_INVALID_SUBSCRIPTION_ID) {
            // Already inactive; nothing to free (no C-side subscription).
            return;
        }
        ovphysx_result_t r = ovphysx_unsubscribe_object_changes(m_id);
        m_id = OVPHYSX_INVALID_SUBSCRIPTION_ID;
        if (r.status == OVPHYSX_API_SUCCESS) {
            m_state.reset();
        } else {
            // Leak: keep m_state alive so any pending C-side callback finds
            // a valid pointer. The leak is permanent for this subscription.
            (void)m_state.release();
        }
    }

    // State is the heap-allocated owner of the std::function callbacks. The
    // C trampolines below receive a State* via the C user_data pointer and
    // dispatch through it. Public so namespace-scope trampolines can reach the
    // member; users should not touch this struct directly.
    struct State {
        ObjectChangeCallbacks callbacks;
    };

private:
    friend ObjectChangeSubscription subscribeObjectChanges(ObjectChangeCallbacks);

    ovphysx_subscription_id_t m_id;
    std::unique_ptr<State> m_state;
};

namespace detail {
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
 * `!handle.isActive()`; the underlying C call's error is not exposed through
 * this overload. Subscriptions are process-global -- callbacks fire for
 * events on every ovphysx instance in the process. See the docstring on
 * ovphysx_subscribe_object_changes in ovphysx.h for the full contract.
 */
inline ObjectChangeSubscription subscribeObjectChanges(ObjectChangeCallbacks callbacks)
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
    if (ovphysx_subscribe_object_changes(&c, &id).status != OVPHYSX_API_SUCCESS) {
        sub.m_state.reset();
        return sub;  // m_id stays OVPHYSX_INVALID_SUBSCRIPTION_ID
    }
    sub.m_id = id;
    return sub;
}

} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_OVPHYSX_HPP
