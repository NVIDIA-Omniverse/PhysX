// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#ifndef OVPHYSX_EXPERIMENTAL_PHYSX_HPP
#define OVPHYSX_EXPERIMENTAL_PHYSX_HPP

//
// C++17 RAII wrapper for ovphysx instance handle
// Provides automatic cleanup and thin C++ wrappers for all C API functions
//

#include <string>
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
 *   PhysX physx;
 *   PhysX::create(physx);
 *   physx.step(0.01f, 0.0f);
 *   physx.waitAll();
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
    
    /// Add a USD file to the stage
    ovphysx_api_status_t addUsd(const std::string& path, const std::string& prefix,
                                 ovphysx_usd_handle_t& out_handle);
    
    /// Remove a previously added USD file
    ovphysx_api_status_t removeUsd(ovphysx_usd_handle_t usd_handle);
    
    /// Reset the stage to empty
    ovphysx_api_status_t reset();
    
    /// Get the USD stage ID
    ovphysx_api_status_t getStageId(int64_t& out_stage_id);

    /// Clone a USD prim hierarchy to create multiple runtime copies (asynchronous).
    /// 
    /// Creates physics-optimized clones in the internal representation for high-performance simulation.
    /// The source prim must exist in the loaded USD stage and have physics properties.
    /// Use waitOp(op_index) or waitAll() to wait for completion before using cloned objects outside stream order.
    /// 
    /// Clones are created using an optimized internal replication system, providing optimal
    /// performance for RL training scenarios with many parallel environments.
    /// 
    /// @param sourcePath USD path of the source prim hierarchy (e.g., "/World/env0")
    /// @param targetPaths Vector of USD paths for cloned hierarchies (e.g., ["/World/env1", "/World/env2"])
    /// @param parentTransforms World-space placement for each cloned environment's
    ///        parent Xform prim.  Flat array of [targetPaths.size() * 7] floats:
    ///        (px, py, pz, qx, qy, qz, qw) per target.  Pass nullptr for identity.
    /// @return OVPHYSX_API_SUCCESS if cloning was queued successfully, OVPHYSX_API_ERROR on error
    ovphysx_api_status_t clone(const std::string& sourcePath, const std::vector<std::string>& targetPaths,
                               const float* parentTransforms = nullptr);


    //------------------------------------------------------------------------------------------------------------
    // Simulation
    //------------------------------------------------------------------------------------------------------------
    
    /// Enqueue a physics simulation step
    ovphysx_api_status_t step(float step_dt, float current_time);
    
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
     * Creates a binding that connects USD prim paths (matched by pattern) to a tensor type,
     * enabling efficient bulk read/write of simulation state.
     *
     * @param out_binding  Receives the created TensorBinding on success
     * @param pattern      USD prim path pattern (e.g., "/World/robot*")
     * @param tensor_type  The type of tensor data to bind
     * @return OVPHYSX_API_SUCCESS on success
     */
    ovphysx_api_status_t createTensorBinding(
        TensorBinding& out_binding,
        const std::string& pattern,
        ovphysx_tensor_type_t tensor_type);

    //------------------------------------------------------------------------------------------------------------
    // Remote storage credentials (process-wide)
    //------------------------------------------------------------------------------------------------------------

    /**
     * @brief Configure S3 credentials for remote USD loading via HTTPS S3 URLs.
     *
     * Process-global — affects all instances. Call before addUsd() with an S3 HTTPS URL.
     * An ovphysx instance must exist so the runtime is loaded.
     *
     * @param host              S3 endpoint (e.g., "my-bucket.s3.us-east-1.amazonaws.com")
     * @param bucket            Bucket name
     * @param region            AWS region (e.g., "us-east-1")
     * @param access_key_id     AWS access key ID
     * @param secret_access_key AWS secret access key
     * @param session_token     STS session token (empty string if unused)
     * @return OVPHYSX_API_SUCCESS on success
     */
    static ovphysx_api_status_t configureS3(
        const std::string& host,
        const std::string& bucket,
        const std::string& region,
        const std::string& access_key_id,
        const std::string& secret_access_key,
        const std::string& session_token = "");

    /**
     * @brief Configure an Azure SAS token for remote USD loading via Azure Blob Storage.
     *
     * Process-global — affects all instances. Call before addUsd() with an Azure Blob URL.
     * An ovphysx instance must exist so the runtime is loaded.
     *
     * @param host      Azure Blob host (e.g., "myaccount.blob.core.windows.net")
     * @param container Container name
     * @param sas_token SAS token string (without leading '?')
     * @return OVPHYSX_API_SUCCESS on success
     */
    static ovphysx_api_status_t configureAzureSas(
        const std::string& host,
        const std::string& container,
        const std::string& sas_token);

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
        auto r = ovphysx_get_physx_ptr(m_handle, primPath.c_str(),
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
        auto r = ovphysx_get_physx_ptr(m_handle, primPath.c_str(), type, &out);
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
     * @brief Factory method to create a PhysX instance with typed config entries.
     *
     * Example:
     * @code
     *   #include "ovphysx/ovphysx_config.h"
     *
     *   ovphysx_config_entry_t entries[] = {
     *       ovphysx_config_entry_disable_contact_processing(true),
     *       ovphysx_config_entry_num_threads(4),
     *   };
     *   PhysX physx;
     *   auto status = PhysX::create(physx, entries, 2);
     *   if (status != OVPHYSX_API_SUCCESS) { ... handle error ... }
     * @endcode
     *
     * Rationale: ovphysx_create_instance can fail; the factory returns status
     * without constructing an invalid object or relying on exceptions. If you
     * prefer a different pattern, wrap this in your own expected/optional logic.
     */
    static ovphysx_api_status_t create(
        PhysX& out_instance,
        const ovphysx_config_entry_t* config_entries = nullptr,
        uint32_t config_entry_count = 0);

private:
    ovphysx_handle_t m_handle;
};

} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_PHYSX_HPP
