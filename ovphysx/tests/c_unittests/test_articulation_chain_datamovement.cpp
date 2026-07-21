// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Test for articulation chain with multiple links and revolute joints.
// This test demonstrates tensor binding API usage for batch reading/writing
// articulation DOF data using the synchronous tensor API.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <cmath>
#include <vector>

using namespace test_utils;

TEST_F(PhysXTestFixture, ArticulationChainDataMovement_CompleteSimulation) {
    // Load additional USD for this test
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/links_chain_sample.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    // Create tensor bindings for articulation chain
    // Use tensor binding API for DOF positions, velocities, and position targets

    // DOF position binding (read joint positions)
    ovphysx_tensor_binding_handle_t dof_pos_binding = 0;
    ovphysx_tensor_binding_desc_t dof_pos_desc = {};
    dof_pos_desc.pattern = make_ovx_string("/World/articulation");
    dof_pos_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &dof_pos_desc, &dof_pos_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // DOF velocity binding (read joint velocities)
    ovphysx_tensor_binding_handle_t dof_vel_binding = 0;
    ovphysx_tensor_binding_desc_t dof_vel_desc = {};
    dof_vel_desc.pattern = make_ovx_string("/World/articulation");
    dof_vel_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32;

    result = ovphysx_create_tensor_binding(m_handle, &dof_vel_desc, &dof_vel_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // DOF position target binding (write control targets)
    ovphysx_tensor_binding_handle_t dof_target_binding = 0;
    ovphysx_tensor_binding_desc_t dof_target_desc = {};
    dof_target_desc.pattern = make_ovx_string("/World/articulation");
    dof_target_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    result = ovphysx_create_tensor_binding(m_handle, &dof_target_desc, &dof_target_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Rigid body pose binding for link transforms
    ovphysx_tensor_binding_handle_t rb_binding = 0;
    ovphysx_tensor_binding_desc_t rb_desc = {};
    rb_desc.pattern = make_ovx_string("/World/articulation/articulationLink*");
    rb_desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    result = ovphysx_create_tensor_binding(m_handle, &rb_desc, &rb_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Query tensor specs to get dimensions
    ovphysx_tensor_spec_t dof_spec, rb_spec;

    result = ovphysx_get_tensor_binding_spec(m_handle, dof_pos_binding, &dof_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_get_tensor_binding_spec(m_handle, rb_binding, &rb_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Allocate CPU buffers for tensor data
    size_t dof_count = dof_spec.shape[0];
    size_t dof_components = dof_spec.shape[1];
    size_t rb_count = rb_spec.shape[0];
    size_t rb_components = rb_spec.shape[1];

    std::vector<float> dof_positions(dof_count * dof_components);
    std::vector<float> dof_velocities(dof_count * dof_components);
    std::vector<float> dof_targets(dof_count * dof_components);
    std::vector<float> rb_poses(rb_count * rb_components);

    // Create DLTensor wrappers for CPU memory
    int64_t dof_shape[2] = {(int64_t)dof_count, (int64_t)dof_components};
    int64_t rb_shape[2] = {(int64_t)rb_count, (int64_t)rb_components};

    DLTensor dof_pos_tensor = {};
    dof_pos_tensor.data = dof_positions.data();
    dof_pos_tensor.device = {kDLCPU, 0};
    dof_pos_tensor.ndim = 2;
    dof_pos_tensor.dtype = {kDLFloat, 32, 1};
    dof_pos_tensor.shape = dof_shape;
    dof_pos_tensor.strides = nullptr;
    dof_pos_tensor.byte_offset = 0;

    DLTensor dof_vel_tensor = {};
    dof_vel_tensor.data = dof_velocities.data();
    dof_vel_tensor.device = {kDLCPU, 0};
    dof_vel_tensor.ndim = 2;
    dof_vel_tensor.dtype = {kDLFloat, 32, 1};
    dof_vel_tensor.shape = dof_shape;
    dof_vel_tensor.strides = nullptr;
    dof_vel_tensor.byte_offset = 0;

    DLTensor dof_target_tensor = {};
    dof_target_tensor.data = dof_targets.data();
    dof_target_tensor.device = {kDLCPU, 0};
    dof_target_tensor.ndim = 2;
    dof_target_tensor.dtype = {kDLFloat, 32, 1};
    dof_target_tensor.shape = dof_shape;
    dof_target_tensor.strides = nullptr;
    dof_target_tensor.byte_offset = 0;

    DLTensor rb_pose_tensor = {};
    rb_pose_tensor.data = rb_poses.data();
    rb_pose_tensor.device = {kDLCPU, 0};
    rb_pose_tensor.ndim = 2;
    rb_pose_tensor.dtype = {kDLFloat, 32, 1};
    rb_pose_tensor.shape = rb_shape;
    rb_pose_tensor.strides = nullptr;
    rb_pose_tensor.byte_offset = 0;

    // Simulation loop: oscillate joint position targets and monitor link positions
    const int TOTAL_STEPS = 500;
    const int STEPS_PER_TARGET_SWITCH = 100;
    const int STEPS_PER_READ = 50;

    const float dt = 1.0f / 60.0f;

    for (int step = 0; step < TOTAL_STEPS; ++step)
    {
        // Write position targets every STEPS_PER_TARGET_SWITCH steps
        if (step % STEPS_PER_TARGET_SWITCH == 0)
        {
            float target_pos = ((step / STEPS_PER_TARGET_SWITCH) % 2 == 0) ? 0.3f : -0.3f;

            // Set all DOF position targets
            for (size_t i = 0; i < dof_count * dof_components; ++i) {
                dof_targets[i] = target_pos;
            }

            result = ovphysx_write_tensor_binding(m_handle, dof_target_binding, &dof_target_tensor, nullptr);
            ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        }

        // Step simulation
        ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, dt);
        ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);

        // Wait for step to complete
        ovphysx_op_wait_result_t wait_result = {};
        ovphysx_result_t step_wait_status = ovphysx_wait_op(m_handle, step_result.op_index, 30000000000ULL, &wait_result);
        ASSERT_EQ(step_wait_status.status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(wait_result.num_errors, 0);
        ovphysx_destroy_wait_result(&wait_result);

        // Read DOF positions and link poses periodically
        if (step > 0 && step % STEPS_PER_READ == 0)
        {
            result = ovphysx_read_tensor_binding(m_handle, dof_pos_binding, &dof_pos_tensor);
            ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

            result = ovphysx_read_tensor_binding(m_handle, rb_binding, &rb_pose_tensor);
            ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        }
    }

    // Final validation - read all link poses and DOF state
    result = ovphysx_read_tensor_binding(m_handle, rb_binding, &rb_pose_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_read_tensor_binding(m_handle, dof_pos_binding, &dof_pos_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_read_tensor_binding(m_handle, dof_vel_binding, &dof_vel_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Verify some motion occurred (check if last link moved from origin)
    // Poses are stored as: [pos_x, pos_y, pos_z, quat_x, quat_y, quat_z, quat_w]
    size_t last_link_idx = rb_count - 1;
    size_t pose_offset = last_link_idx * rb_components;
    float last_link_x = rb_poses[pose_offset + 0];
    float last_link_y = rb_poses[pose_offset + 1];
    float last_link_z = rb_poses[pose_offset + 2];
    float distance_from_origin = std::sqrt(last_link_x*last_link_x + last_link_y*last_link_y + last_link_z*last_link_z);

    EXPECT_GT(distance_from_origin, 0.1f) << "Chain should have moved significantly";

    // Cleanup tensor bindings
    ovphysx_destroy_tensor_binding(m_handle, rb_binding);
    ovphysx_destroy_tensor_binding(m_handle, dof_pos_binding);
    ovphysx_destroy_tensor_binding(m_handle, dof_vel_binding);
    ovphysx_destroy_tensor_binding(m_handle, dof_target_binding);

    // Cleanup - reset to clear USD
    ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS);

    ovphysx_op_wait_result_t reset_wait_result = {};
    ovphysx_result_t reset_wait_status = ovphysx_wait_op(m_handle, reset_result.op_index, 30000000000ULL, &reset_wait_result);
    ASSERT_EQ(reset_wait_status.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(reset_wait_result.num_errors, 0);
    ovphysx_destroy_wait_result(&reset_wait_result);
}
