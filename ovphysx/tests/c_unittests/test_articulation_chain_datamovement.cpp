// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


// Test for an articulation chain with multiple links and revolute joints. Drives joint POSITION
// targets through the write session and reads link poses back through the read session, verifying
// the chain moves significantly over a long run.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace test_utils;

TEST_F(PhysXTestFixture, ArticulationChainDataMovement_CompleteSimulation) {
    const char* usd_path = OVPHYSX_SOURCE_DIR "/tests/data/links_chain_sample.usda";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path));

    // This USD's joint drives are pure dampers (drive:angular:physics:damping, no stiffness), so a
    // jointPositionTarget has no solver effect and only a velocity target drives them. The test
    // commands a constant angular velocity and reads jointVelocity back as the drive-applied control.
    const ovx_string_or_token_t velTargetAttr = {
        0, { OVPHYSX_ATTR_JOINT_VELOCITY_TARGET, sizeof(OVPHYSX_ATTR_JOINT_VELOCITY_TARGET) - 1 }
    };
    const ovx_string_or_token_t jointVelAttr = {
        0, { OVPHYSX_ATTR_JOINT_VELOCITY, sizeof(OVPHYSX_ATTR_JOINT_VELOCITY) - 1 }
    };
    const ovx_string_or_token_t posAttr = { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } };

    const int TOTAL_STEPS = 200;
    const float dt = 1.0f / 60.0f;

    // Read every articulation-link world position into a flat [x,y,z,...] vector. Row order is
    // stable across reads (no structural change here), so two snapshots subtract row-wise.
    auto readLinkPositions = [&]() -> std::vector<float> {
        std::vector<float> out;
        ovphysx_query_handle_t rq = 0;
        EXPECT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_LINK, OVPHYSX_SCOPE_ALL, &rq).status,
                  OVPHYSX_API_SUCCESS);
        ovphysx_read_handle_t rs = 0;
        EXPECT_EQ(ovphysx_read(m_handle, rq, &posAttr, 1, &rs).status, OVPHYSX_API_SUCCESS);
        const ovstage_read_group_t* g = nullptr;
        for (;;)
        {
            const ovphysx_result_t fr = ovphysx_fetch_read_next(m_handle, rs, &g);
            if (fr.status == OVPHYSX_API_END_OF_ITERATION)
                break;
            if (fr.status != OVPHYSX_API_SUCCESS)
            {
                ADD_FAILURE() << "ovphysx_fetch_read_next(LINK) failed (status " << static_cast<int>(fr.status) << ")";
                break;  // g is NULL on a non-success status and must not be dereferenced
            }
            for (uint32_t ti = 0; !g->is_delete && g->data.tensors && ti < g->data.tensor_count; ++ti)
            {
                const DLTensor& t = g->data.tensors[ti];
                if (!t.data || t.device.device_type != kDLCPU || t.dtype.lanes != 3)
                    continue;
                const int64_t rows = t.shape[0];
                const float* p = static_cast<const float*>(t.data);
                out.insert(out.end(), p, p + rows * 3);
            }
            ovphysx_release_group(m_handle, rs, g->read_group_id);
        }
        ovphysx_release_read(m_handle, rs);
        ovphysx_release_query(m_handle, rq);
        return out;
    };

    auto stepN = [&](int n) {
        for (int i = 0; i < n; ++i)
        {
            const ovphysx_enqueue_result_t sr = ovphysx_step(m_handle, dt);
            EXPECT_EQ(sr.status, OVPHYSX_API_SUCCESS);
            ovphysx_op_wait_result_t wr = {};
            EXPECT_EQ(ovphysx_wait_op(m_handle, sr.op_index, 30000000000ULL, &wr).status, OVPHYSX_API_SUCCESS);
            EXPECT_EQ(wr.num_errors, 0);
            ovphysx_destroy_wait_result(&wr);
        }
    };

    // Read every joint's DOF velocity into a flat vector (per-axis).
    auto readJointVelocities = [&]() -> std::vector<float> {
        std::vector<float> out;
        ovphysx_query_handle_t rq = 0;
        EXPECT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &rq).status,
                  OVPHYSX_API_SUCCESS);
        ovphysx_read_handle_t rs = 0;
        EXPECT_EQ(ovphysx_read(m_handle, rq, &jointVelAttr, 1, &rs).status, OVPHYSX_API_SUCCESS);
        const ovstage_read_group_t* g = nullptr;
        for (;;)
        {
            const ovphysx_result_t fr = ovphysx_fetch_read_next(m_handle, rs, &g);
            if (fr.status == OVPHYSX_API_END_OF_ITERATION)
                break;
            if (fr.status != OVPHYSX_API_SUCCESS)
            {
                ADD_FAILURE() << "ovphysx_fetch_read_next(JOINT) failed (status " << static_cast<int>(fr.status) << ")";
                break;  // g is NULL on a non-success status and must not be dereferenced
            }
            for (uint32_t ti = 0; !g->is_delete && g->data.tensors && ti < g->data.tensor_count; ++ti)
            {
                const DLTensor& t = g->data.tensors[ti];
                if (!t.data || t.device.device_type != kDLCPU)
                    continue;
                const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
                const size_t count = static_cast<size_t>(t.shape[0]) * lanes;
                const float* p = static_cast<const float*>(t.data);
                out.insert(out.end(), p, p + count);
            }
            ovphysx_release_group(m_handle, rs, g->read_group_id);
        }
        ovphysx_release_read(m_handle, rs);
        ovphysx_release_query(m_handle, rq);
        return out;
    };

    // Warm up one step so link poses resolve, then snapshot the chain at rest before driving it.
    stepN(1);
    const std::vector<float> initial_positions = readLinkPositions();
    ASSERT_FALSE(initial_positions.empty()) << "No link positions read after warmup";

    // Command every joint to rotate at a constant angular velocity (jointVelocityTarget, deg/s).
    // The large damping tracks it, but the joints are limited to +/-5.625 deg, so they reach the
    // limit within a handful of steps.
    const float DRIVE_DEG_PER_S = 60.0f;
    {
        ovphysx_query_handle_t wq = 0;
        ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &wq).status,
                  OVPHYSX_API_SUCCESS);
        ovphysx_write_handle_t ws = 0;
        ASSERT_EQ(ovphysx_write(m_handle, wq, &velTargetAttr, &ws).status, OVPHYSX_API_SUCCESS);
        const ovstage_map_group_t* g = nullptr;
        int groups_written = 0;
        for (ovphysx_result_t fw;
             (fw = ovphysx_fetch_write_next(m_handle, ws, &g)).status != OVPHYSX_API_END_OF_ITERATION;)
        {
            ASSERT_EQ(fw.status, OVPHYSX_API_SUCCESS);  // any non-END_OF_ITERATION status is a real error
            for (uint32_t ti = 0; g->data.tensors && ti < g->data.tensor_count; ++ti)
            {
                const DLTensor& t = g->data.tensors[ti];
                ASSERT_TRUE(t.data && t.device.device_type == kDLCPU);  // fill EVERY entry before commit
                const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
                const size_t count = static_cast<size_t>(t.shape[0]) * lanes;
                float* dst = static_cast<float*>(t.data);
                for (size_t j = 0; j < count; ++j)
                    dst[j] = DRIVE_DEG_PER_S;
            }
            ASSERT_EQ(ovphysx_commit_group(m_handle, ws, g, ovstage_cuda_sync_t{}).status, OVPHYSX_API_SUCCESS);
            ++groups_written;
        }
        ASSERT_GT(groups_written, 0) << "write session produced no groups";
        ovphysx_release_write(m_handle, ws);
        ovphysx_release_query(m_handle, wq);
    }

    // Step a couple of frames so the joints spin up while still short of their limits, then read
    // jointVelocity back. With the drive the joints track ~60 deg/s, without it they show only the
    // ~7 deg/s gravity transient, so the 30 deg/s threshold fails the test if the write did not
    // reach the solver. A link-displacement check alone cannot tell drive from gravity here, since
    // a pure damper with no stiffness lets gravity droop the chain to the same +/-5.625 deg limits.
    stepN(2);
    const std::vector<float> joint_vel = readJointVelocities();
    ASSERT_FALSE(joint_vel.empty()) << "no joint velocities read";
    float max_abs_vel = 0.0f;
    for (float v : joint_vel)
        max_abs_vel = std::max(max_abs_vel, std::abs(v));
    ASSERT_GT(max_abs_vel, 30.0f)
        << "joints are not driven; jointVelocityTarget had no solver effect (max |vel| = "
        << max_abs_vel << " deg/s, expected ~" << DRIVE_DEG_PER_S << ")";

    // Run the rest so the chain settles into its driven (limit-bent) configuration.
    stepN(TOTAL_STEPS);

    // Compare each link's final world position to its rest snapshot and take the largest
    // displacement. Distance from the world origin would not do, because the links are authored
    // ~22-28 units out and |position| clears any threshold even when frozen.
    const std::vector<float> final_positions = readLinkPositions();
    ASSERT_EQ(final_positions.size(), initial_positions.size());
    float max_displacement = 0.0f;
    for (size_t i = 0; i + 2 < final_positions.size(); i += 3)
    {
        const float dx = final_positions[i + 0] - initial_positions[i + 0];
        const float dy = final_positions[i + 1] - initial_positions[i + 1];
        const float dz = final_positions[i + 2] - initial_positions[i + 2];
        max_displacement = std::max(max_displacement, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    EXPECT_GT(max_displacement, 0.1f) << "Chain should have moved significantly under the drive";

    // Reset to clear the USD stage.
    ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset_result.status, OVPHYSX_API_SUCCESS);
    ovphysx_op_wait_result_t reset_wait_result = {};
    ASSERT_EQ(ovphysx_wait_op(m_handle, reset_result.op_index, 30000000000ULL, &reset_wait_result).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(reset_wait_result.num_errors, 0);
    ovphysx_destroy_wait_result(&reset_wait_result);
}
