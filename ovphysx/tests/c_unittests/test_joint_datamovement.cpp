// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


// Test that loads an articulation scene and uses the session read/write API to:
// - Write joint velocity targets via ovphysx_write / fetch_write_next / commit_group
// - Read joint velocities via ovphysx_read / fetch_read_next
// - Step the simulation and verify the articulation moves as expected

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <vector>
#include <cmath>

using namespace test_utils;

TEST_F(PhysXTestFixture, JointDataMovement_ArticulationDofSessionRW) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/links_chain_sample.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    // Session-API attributes: write joint velocity TARGETS (app->physics), read joint
    // velocities back (physics->app). ARTICULATION_JOINT groups carry per-axis columns.
    const ovx_string_or_token_t velTargetAttr = {
        0, { OVPHYSX_ATTR_JOINT_VELOCITY_TARGET, sizeof(OVPHYSX_ATTR_JOINT_VELOCITY_TARGET) - 1 }
    };
    const ovx_string_or_token_t velAttr = { 0, { OVPHYSX_ATTR_JOINT_VELOCITY, sizeof(OVPHYSX_ATTR_JOINT_VELOCITY) - 1 } };

    for (int i = 0; i < 100; ++i)
    {
        // Step first: DirectGPU (and the tensor backend's scene view) refuse a read or write session
        // before the first step. A velocity target written after a step drives the following one.
        ovphysx_enqueue_result_t step_result = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step_result.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(waitForOperationSuccess(m_handle, step_result.op_index));

        if (i % 5 == 0)
        {
            // Write an oscillating velocity target onto every joint axis via the write session. The
            // joint group emits one tensor per joint, so fill each, not just the first.
            // jointVelocityTarget is in the session API's angular units (deg/s). The amplitude of
            // 360 deg/s equals 2*pi rad/s.
            const float target = 360.0f * std::sin(i * 2.0f * 3.14159f / 60.0f);
            ovphysx_query_handle_t writeQuery = 0;
            ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &writeQuery).status,
                      OVPHYSX_API_SUCCESS);
            ovphysx_write_handle_t writeSession = 0;
            ASSERT_EQ(ovphysx_write(m_handle, writeQuery, &velTargetAttr, &writeSession).status, OVPHYSX_API_SUCCESS);

            const ovstage_map_group_t* group = nullptr;
            int groups_written = 0;
            for (ovphysx_result_t fw;
                 (fw = ovphysx_fetch_write_next(m_handle, writeSession, &group)).status != OVPHYSX_API_END_OF_ITERATION;)
            {
                // Any status other than END_OF_ITERATION is a real error, not exhaustion.
                ASSERT_EQ(fw.status, OVPHYSX_API_SUCCESS);
                for (uint32_t ti = 0; group->data.tensors && ti < group->data.tensor_count; ++ti)
                {
                    const DLTensor& t = group->data.tensors[ti];
                    // Commit publishes every entry, so every tensor must be filled before the
                    // group is committed. This is a CPU write session.
                    ASSERT_TRUE(t.data && t.device.device_type == kDLCPU);
                    const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
                    const size_t count = static_cast<size_t>(t.shape[0]) * lanes;
                    float* dst = static_cast<float*>(t.data);
                    for (size_t j = 0; j < count; ++j)
                        dst[j] = target;
                }
                ASSERT_EQ(ovphysx_commit_group(m_handle, writeSession, group, ovstage_cuda_sync_t{}).status,
                          OVPHYSX_API_SUCCESS);
                ++groups_written;
            }
            ASSERT_GT(groups_written, 0) << "write session produced no groups";
            ovphysx_release_write(m_handle, writeSession);
            ovphysx_release_query(m_handle, writeQuery);
        }

        // Read joint velocities back through the read session and verify the drive took.
        if (i % 10 == 0 && i > 0)
        {
            ovphysx_query_handle_t readQuery = 0;
            ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &readQuery).status,
                      OVPHYSX_API_SUCCESS);
            ovphysx_read_handle_t readSession = 0;
            ASSERT_EQ(ovphysx_read(m_handle, readQuery, &velAttr, 1, &readSession).status, OVPHYSX_API_SUCCESS);

            bool has_nonzero_velocity = false;
            const ovstage_read_group_t* group = nullptr;
            for (;;)
            {
                const ovphysx_result_t fr = ovphysx_fetch_read_next(m_handle, readSession, &group);
                if (fr.status == OVPHYSX_API_END_OF_ITERATION)
                    break;
                ASSERT_EQ(fr.status, OVPHYSX_API_SUCCESS);
                for (uint32_t ti = 0; !group->is_delete && group->data.tensors && ti < group->data.tensor_count; ++ti)
                {
                    const DLTensor& t = group->data.tensors[ti];
                    if (!t.data || t.device.device_type != kDLCPU)
                        continue;
                    const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
                    const size_t count = static_cast<size_t>(t.shape[0]) * lanes;
                    const float* d = static_cast<const float*>(t.data);
                    for (size_t j = 0; j < count; ++j)
                        // jointVelocity is deg/s (session units). The threshold is 0.01 rad/s
                        // expressed in deg/s. A threshold of 0.01 deg/s would be ~57x weaker and
                        // latch on solver jitter.
                        if (std::abs(d[j]) > 0.573f)
                            has_nonzero_velocity = true;
                }
                ovphysx_release_group(m_handle, readSession, group->read_group_id);
            }
            ovphysx_release_read(m_handle, readSession);
            ovphysx_release_query(m_handle, readQuery);
            EXPECT_TRUE(has_nonzero_velocity) << "Expected non-zero velocities after " << i << " steps";
        }
    }

    // After 100 driven steps the chain's joints have displaced toward their +/-5.625 deg limits, so
    // a jointPosition read must show non-trivial angles.
    {
        const ovx_string_or_token_t posAttr = { 0, { OVPHYSX_ATTR_JOINT_POSITION, sizeof(OVPHYSX_ATTR_JOINT_POSITION) - 1 } };
        ovphysx_query_handle_t posQuery = 0;
        ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &posQuery).status,
                  OVPHYSX_API_SUCCESS);
        ovphysx_read_handle_t posSession = 0;
        ASSERT_EQ(ovphysx_read(m_handle, posQuery, &posAttr, 1, &posSession).status, OVPHYSX_API_SUCCESS);
        float max_abs_pos = 0.0f;
        const ovstage_read_group_t* group = nullptr;
        for (;;)
        {
            const ovphysx_result_t fr = ovphysx_fetch_read_next(m_handle, posSession, &group);
            if (fr.status == OVPHYSX_API_END_OF_ITERATION)
                break;
            ASSERT_EQ(fr.status, OVPHYSX_API_SUCCESS);
            for (uint32_t ti = 0; !group->is_delete && group->data.tensors && ti < group->data.tensor_count; ++ti)
            {
                const DLTensor& t = group->data.tensors[ti];
                if (!t.data || t.device.device_type != kDLCPU)
                    continue;
                const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
                const size_t count = static_cast<size_t>(t.shape[0]) * lanes;
                const float* d = static_cast<const float*>(t.data);
                for (size_t j = 0; j < count; ++j)
                    if (std::abs(d[j]) > max_abs_pos)
                        max_abs_pos = std::abs(d[j]);
            }
            ovphysx_release_group(m_handle, posSession, group->read_group_id);
        }
        ovphysx_release_read(m_handle, posSession);
        ovphysx_release_query(m_handle, posQuery);
        EXPECT_GT(max_abs_pos, 0.5f)
            << "jointPosition read back ~0 (max " << max_abs_pos << " deg); the driven joints should have displaced";
    }

    // Reset to clear the USD stage.
    ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, reset_result.op_index));
}
