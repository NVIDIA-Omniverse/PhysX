// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Test that loads an articulation scene and uses the tensor binding API to:
// - Write DOF velocity targets using ovphysx_write_tensor_binding()
// - Read DOF positions and velocities using ovphysx_read_tensor_binding()
// - Step the simulation and verify the articulation moves as expected

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <vector>
#include <cmath>

using namespace test_utils;

TEST_F(PhysXTestFixture, JointDataMovement_ArticulationDofTensorBinding) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/links_chain_sample.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    // 1. DOF position binding (to read joint positions)
    ovphysx_tensor_binding_handle_t dof_pos_binding = 0;
    ovphysx_tensor_binding_desc_t dof_pos_desc{};
    dof_pos_desc.pattern = make_ovx_string("/World/articulation");
    dof_pos_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &dof_pos_desc, &dof_pos_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // 2. DOF velocity binding (to read joint velocities)
    ovphysx_tensor_binding_handle_t dof_vel_binding = 0;
    ovphysx_tensor_binding_desc_t dof_vel_desc{};
    dof_vel_desc.pattern = make_ovx_string("/World/articulation");
    dof_vel_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32;

    result = ovphysx_create_tensor_binding(m_handle, &dof_vel_desc, &dof_vel_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // 3. DOF velocity target binding (to write control targets)
    ovphysx_tensor_binding_handle_t dof_vel_target_binding = 0;
    ovphysx_tensor_binding_desc_t dof_vel_target_desc{};
    dof_vel_target_desc.pattern = make_ovx_string("/World/articulation");
    dof_vel_target_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32;

    result = ovphysx_create_tensor_binding(m_handle, &dof_vel_target_desc, &dof_vel_target_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Query tensor specs to determine dimensions
    ovphysx_tensor_spec_t dof_spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, dof_pos_binding, &dof_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(dof_spec.ndim, 2);
    ASSERT_GT(dof_spec.shape[0], 0);  // At least one articulation
    ASSERT_GT(dof_spec.shape[1], 0);  // At least one DOF

    size_t num_articulations = static_cast<size_t>(dof_spec.shape[0]);
    size_t num_dofs = static_cast<size_t>(dof_spec.shape[1]);
    size_t total_elements = num_articulations * num_dofs;

    std::vector<float> dof_positions(total_elements, 0.0f);
    std::vector<float> dof_velocities(total_elements, 0.0f);
    std::vector<float> dof_vel_targets(total_elements, 0.0f);

    // Create DLTensor wrappers for CPU memory
    int64_t shape[2] = {static_cast<int64_t>(num_articulations), static_cast<int64_t>(num_dofs)};

    DLTensor pos_tensor{};
    pos_tensor.data = dof_positions.data();
    pos_tensor.device = {kDLCPU, 0};
    pos_tensor.ndim = 2;
    pos_tensor.dtype = {kDLFloat, 32, 1};
    pos_tensor.shape = shape;
    pos_tensor.strides = nullptr;
    pos_tensor.byte_offset = 0;

    DLTensor vel_tensor{};
    vel_tensor.data = dof_velocities.data();
    vel_tensor.device = {kDLCPU, 0};
    vel_tensor.ndim = 2;
    vel_tensor.dtype = {kDLFloat, 32, 1};
    vel_tensor.shape = shape;
    vel_tensor.strides = nullptr;
    vel_tensor.byte_offset = 0;

    DLTensor vel_target_tensor{};
    vel_target_tensor.data = dof_vel_targets.data();
    vel_target_tensor.device = {kDLCPU, 0};
    vel_target_tensor.ndim = 2;
    vel_target_tensor.dtype = {kDLFloat, 32, 1};
    vel_target_tensor.shape = shape;
    vel_target_tensor.strides = nullptr;
    vel_target_tensor.byte_offset = 0;

    for (int i = 0; i < 100; ++i)
    {
        if(i % 5 == 0) {
            // write oscillating target velocity
            for (size_t j = 0; j < total_elements; ++j) {
                dof_vel_targets[j] = 2.0f * 3.14159f * std::sin(i * 2.0f * 3.14159f / 60.0f);
            }
            result = ovphysx_write_tensor_binding(m_handle, dof_vel_target_binding, &vel_target_tensor, nullptr);
            ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        }

        ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f/60.0f);
        ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(waitForOperationSuccess(m_handle, step_result.op_index));

        // Read DOF positions and velocities periodically (tensor API is synchronous)
        if (i % 10 == 0)
        {
            result = ovphysx_read_tensor_binding(m_handle, dof_pos_binding, &pos_tensor);
            ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

            result = ovphysx_read_tensor_binding(m_handle, dof_vel_binding, &vel_tensor);
            ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

            // Optional: Verify data is changing (basic sanity check)
            // After some steps, velocities should be non-zero due to target velocity drive
            if (i > 0) {
                bool has_nonzero_velocity = false;
                for (size_t j = 0; j < total_elements; ++j) {
                    if (std::abs(dof_velocities[j]) > 0.01f) {
                        has_nonzero_velocity = true;
                        break;
                    }
                }
                EXPECT_TRUE(has_nonzero_velocity) << "Expected non-zero velocities after " << i << " steps";
            }
        }
    }

    // Cleanup tensor bindings
    ovphysx_destroy_tensor_binding(m_handle, dof_pos_binding);
    ovphysx_destroy_tensor_binding(m_handle, dof_vel_binding);
    ovphysx_destroy_tensor_binding(m_handle, dof_vel_target_binding);

    // Cleanup - reset to clear USD
    ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, reset_result.op_index));
}
