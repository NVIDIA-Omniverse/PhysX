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
#include <utility>

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_export.h"
#include "ovphysx/experimental/Helpers.hpp"

namespace ovphysx {

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
    // Settings
    //------------------------------------------------------------------------------------------------------------
    
    /// Set a configuration setting
    ovphysx_api_status_t setSetting(const std::string& key, const std::string& value);
    
    /// Get a configuration setting value
    ovphysx_api_status_t getSetting(const std::string& key, std::string& value_out);
    
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
    /// @return OVPHYSX_API_SUCCESS if cloning was queued successfully, OVPHYSX_API_ERROR on error
    ovphysx_api_status_t clone(const std::string& sourcePath, const std::vector<std::string>& targetPaths);


    //------------------------------------------------------------------------------------------------------------
    // Simulation
    //------------------------------------------------------------------------------------------------------------
    
    /// Enqueue a physics simulation step
    ovphysx_api_status_t step(float elapsed_time, float current_time);
    
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
    // Factory
    //------------------------------------------------------------------------------------------------------------
    
    /**
     * @brief Factory method to create a PhysX instance with settings
     * 
     * Example:
     *   ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);  // optional: default is WARNING
     *   PhysX physx;
     *   auto status = PhysX::create(physx, {
     *       {"physics/physxDispatcher", "true"},
     *   });
     *   if (status != OVPHYSX_API_SUCCESS) { ... handle error ... }
     *
     * Rationale: ovphysx_create_instance can fail; the factory returns status
     * without constructing an invalid object or relying on exceptions. If you
     * prefer a different pattern, wrap this in your own expected/optional logic.
     */
    static ovphysx_api_status_t create(
        PhysX& out_instance,
        const std::vector<std::pair<std::string, std::string>>& settings = {});

private:
    ovphysx_handle_t m_handle;
};

} // namespace ovphysx

#endif // OVPHYSX_EXPERIMENTAL_PHYSX_HPP
