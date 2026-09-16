// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-10
 * @maps_to TEST-INPUT-COVERAGE-003
 *
 * Runtime negative control for the CONDITIONAL classification of jointLimit. ovphysx_writability
 * returns OVPHYSX_WRITABILITY_CONDITIONAL for it, not WRITABLE. An axis only has a limit interval
 * when its motion is eLIMITED, and PhysX refuses setMotion() on an in-scene articulation, so a
 * finite limit aimed at a free axis cannot land at all. The session write fails the commit and
 * writes nothing, not even the limited axes it could accept, while writing each axis its own
 * read-back value round-trips as a legal no-op. The backend scatter is covered by the ovruntime
 * doctest TestOvstageWriteScatter.cpp. This test pins the same contract at the public C session API.
 *
 * CartPole mixes a limited prismatic cartJoint and a free revolute poleJoint, so a whole-set finite
 * jointLimit write necessarily hits the free axis. Runs in the CPU pass (no "GpuTest" in the name),
 * so the read-back columns are host tensors.
 */

#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx.h>

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <vector>

using namespace test_utils;

namespace
{

// A string literal carries its length in its type, so the attribute name needs no std::strlen.
template <size_t N>
ovx_string_or_token_t makeAttr(const char (&name)[N])
{
    ovx_string_or_token_t a{};
    a.token = 0;
    a.string.ptr = name;
    a.string.length = N - 1;
    return a;
}

ovstage_cuda_sync_t noSync()
{
    ovstage_cuda_sync_t s{};
    s.stream = 0;
    s.wait_event = 0;
    return s;
}

} // namespace

TEST_F(PhysXTestFixture, JointLimitOnAFreeAxisIsRefusedAndWritesNothing)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/CartPole.usda")) << "Failed to attach CartPole.usda";
    // A session refuses reads/writes before the first step, and the limit getters need a stepped
    // articulation to resolve.
    for (int i = 0; i < 3; ++i)
        ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    const ovx_string_or_token_t limitAttr = makeAttr(OVPHYSX_ATTR_JOINT_LIMIT);

    // Every joint's (low, high) limit into a flat vector, in query order. ARTICULATION_JOINT is an
    // array group with one tensor per joint, so every tensor is read, not just tensors[0].
    auto readLimits = [&]() {
        std::vector<float> out;
        ovphysx_query_handle_t rq = 0;
        EXPECT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &rq).status,
                  OVPHYSX_API_SUCCESS);
        ovphysx_read_handle_t rh = 0;
        EXPECT_EQ(ovphysx_read(m_handle, rq, &limitAttr, 1, &rh).status, OVPHYSX_API_SUCCESS);
        const ovstage_read_group_t* g = nullptr;
        while (ovphysx_fetch_read_next(m_handle, rh, &g).status == OVPHYSX_API_SUCCESS)
        {
            for (uint32_t ti = 0; g->data.tensors && ti < g->data.tensor_count; ++ti)
            {
                const DLTensor& t = g->data.tensors[ti];
                if (!t.data || t.device.device_type != kDLCPU)
                    continue;
                const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
                const size_t n = static_cast<size_t>(t.shape[0]) * lanes;
                const float* p = static_cast<const float*>(t.data);
                out.insert(out.end(), p, p + n);
            }
            ovphysx_release_group(m_handle, rh, g->read_group_id);
        }
        ovphysx_release_read(m_handle, rh);
        ovphysx_release_query(m_handle, rq);
        return out;
    };

    // Fill every jointLimit float from valueAt(flatIndex), in the same flat order readLimits
    // produces, and commit each group. Returns the worst commit status, which is non-SUCCESS if any
    // group's commit was refused. Every entry is filled before committing, so no group is published
    // half-filled.
    auto writeLimits = [&](auto valueAt) {
        ovphysx_query_handle_t q = 0;
        EXPECT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_ARTICULATION_JOINT, OVPHYSX_SCOPE_ALL, &q).status,
                  OVPHYSX_API_SUCCESS);
        ovphysx_write_handle_t w = 0;
        EXPECT_EQ(ovphysx_write(m_handle, q, &limitAttr, &w).status, OVPHYSX_API_SUCCESS);
        ovphysx_result_t worst{ OVPHYSX_API_SUCCESS };
        size_t flat = 0;
        const ovstage_map_group_t* g = nullptr;
        while (ovphysx_fetch_write_next(m_handle, w, &g).status == OVPHYSX_API_SUCCESS)
        {
            for (uint32_t ti = 0; g->data.tensors && ti < g->data.tensor_count; ++ti)
            {
                const DLTensor& t = g->data.tensors[ti];
                if (!t.data || t.device.device_type != kDLCPU)
                    continue;
                const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
                const size_t n = static_cast<size_t>(t.shape[0]) * lanes;
                float* p = static_cast<float*>(t.data);
                for (size_t j = 0; j < n; ++j)
                    p[j] = valueAt(flat++);
            }
            const ovphysx_result_t c = ovphysx_commit_group(m_handle, w, g, noSync());
            if (c.status != OVPHYSX_API_SUCCESS)
                worst = c;
        }
        ovphysx_release_write(m_handle, w);
        ovphysx_release_query(m_handle, q);
        return worst;
    };

    const std::vector<float> before = readLimits();
    ASSERT_FALSE(before.empty()) << "no jointLimit column read from CartPole";
    // The fixture must expose a free axis, or the condition is never exercised. A free axis reads
    // the +/-FLT_MAX unlimited sentinel.
    const float kUnlimited = std::numeric_limits<float>::max();
    ASSERT_NE(std::find(before.begin(), before.end(), kUnlimited), before.end())
        << "CartPole must expose a free (unlimited) axis for this negative control";

    // A finite limit on every DOF necessarily hits the free axis, so the commit is refused.
    const ovphysx_result_t finite = writeLimits([](size_t j) { return (j % 2 == 0) ? -30.0f : 45.0f; });
    EXPECT_NE(finite.status, OVPHYSX_API_SUCCESS)
        << "a finite jointLimit on a free axis must fail the commit (REQ-INPUT-COVERAGE-001 AC-10)";
    // Nothing landed, not even on the limited axis the write could have accepted.
    EXPECT_EQ(readLimits(), before)
        << "a refused jointLimit write must leave every axis unchanged, not half-apply (AC-10)";

    // Writing each axis its own read-back value (the free axis its +/-FLT_MAX sentinel, the limited
    // axis its interval) is a legal no-op and commits.
    const ovphysx_result_t roundtrip =
        writeLimits([&](size_t j) { return j < before.size() ? before[j] : kUnlimited; });
    EXPECT_EQ(roundtrip.status, OVPHYSX_API_SUCCESS)
        << "writing the read-back limits back must succeed as a no-op (AC-10)";
    EXPECT_EQ(readLimits(), before) << "the no-op round-trip must not change any limit";

    ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForOperationSuccess(m_handle, reset.op_index));
}
