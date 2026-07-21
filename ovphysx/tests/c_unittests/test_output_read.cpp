// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <string>
#include <vector>

using namespace test_utils;

namespace
{
void stepN(ovphysx_handle_t handle, int n)
{
    for (int i = 0; i < n; ++i)
        ASSERT_EQ(ovphysx_step_sync(handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
}
} // namespace

class OutputReadTest : public PhysXTestFixture
{
};

// Query rigid bodies and read position + orientation as faithful column groups.
TEST_F(OutputReadTest, RigidBodyPositionAndOrientation)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to attach ovstage USD";

    stepN(m_handle, 10); // let the boxes fall

    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(query, 0u) << "expected at least one rigid body in the scene";

    // Discovery: the query reports its matched prim count + produced attributes
    // (ovstage's own ovstage_query_result_t).
    ovstage_query_result_t qr{};
    ASSERT_EQ(ovphysx_fetch_query_result(m_handle, query, &qr).status, OVPHYSX_API_SUCCESS);
    EXPECT_GT(qr.total_prim_count, 0u);

    // The interned identifiers resolve through the shared ovstage path dictionary.
    void* dict = nullptr;
    EXPECT_EQ(ovphysx_query_shared_dictionary(m_handle, query, &dict).status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(dict, nullptr);

    const ovx_string_or_token_t names[] = {
        { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
        { 0, { OVPHYSX_ATTR_ORIENTATION, sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
    };
    ovphysx_read_handle_t read = 0;
    ASSERT_EQ(ovphysx_read(m_handle, query, names, 2, &read).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(read, 0u);

    int groups = 0;
    int64_t posRows = 0;
    int64_t oriRows = 0;
    for (;;)
    {
        // Producer-owned group: fetch hands back a borrowed ovstage_read_group_t*.
        const ovstage_read_group_t* gp = nullptr;
        const ovphysx_result_t r = ovphysx_fetch_read_next(m_handle, read, &gp);
        if (r.status == OVPHYSX_API_END_OF_ITERATION)
            break;
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(gp, nullptr);
        const ovstage_read_group_t& g = *gp;
        ++groups;
        EXPECT_NE(g.attribute, 0u);                 // interned attribute token
        EXPECT_FALSE(g.is_array);
        EXPECT_GT(g.prims.count, 0u);
        EXPECT_NE(g.prims.list, 0u);                // interned prim-list handle
        ASSERT_GE(g.data.tensor_count, 1u);
        ASSERT_NE(g.data.tensors, nullptr);
        const DLTensor& t = g.data.tensors[0];
        ASSERT_NE(t.shape, nullptr);
        ASSERT_GE(t.ndim, 1);
        // Faithful: a fixed column stacks all prims along shape[0]; tuple width is
        // carried in dtype.lanes (vec3 / quat), NOT a trailing shape dim.
        if (t.dtype.lanes == 3)
            posRows = t.shape[0]; // position
        else if (t.dtype.lanes == 4)
            oriRows = t.shape[0]; // orientation (quat)
        // Group storage is stable until released by id.
        EXPECT_EQ(ovphysx_release_group(m_handle, read, g.read_group_id).status, OVPHYSX_API_SUCCESS);
    }

    EXPECT_GE(groups, 2);                                  // at least a position + an orientation group
    EXPECT_GT(posRows, 0);                                 // at least one rigid body
    EXPECT_EQ(posRows, oriRows);                           // same body set in both columns
    EXPECT_EQ((int64_t)qr.total_prim_count, posRows);      // discovery count matches the column rows

    EXPECT_EQ(ovphysx_release_read(m_handle, read).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_release_query(m_handle, query).status, OVPHYSX_API_SUCCESS);
}

// Null-argument and lifecycle edge cases (C-ABI guarantees).
TEST_F(OutputReadTest, ArgumentValidation)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    stepN(m_handle, 1);

    // null out_query
    EXPECT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, nullptr).status,
              OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(ovphysx_query(m_handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query).status,
              OVPHYSX_API_SUCCESS);

    // null out args on discovery / dictionary / read / fetch
    EXPECT_EQ(ovphysx_fetch_query_result(m_handle, query, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_query_shared_dictionary(m_handle, query, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_read_handle_t read = 0;
    const ovx_string_or_token_t names[] = {
        { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
    };
    EXPECT_EQ(ovphysx_read(m_handle, query, names, 1, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);
    ASSERT_EQ(ovphysx_read(m_handle, query, names, 1, &read).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_fetch_read_next(m_handle, read, nullptr).status, OVPHYSX_API_INVALID_ARGUMENT);

    // release_group is idempotent for an unknown id; release_read/query idempotent.
    EXPECT_EQ(ovphysx_release_group(m_handle, read, 123456789ull).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_release_read(m_handle, read).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_release_read(m_handle, read).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_release_query(m_handle, query).status, OVPHYSX_API_SUCCESS);
}
