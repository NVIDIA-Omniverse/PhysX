// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-DEBUG-RENDER-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-6 AC-7 AC-8 AC-9
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include "debug_render_session_shim.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <cmath>
#include <limits>

namespace
{
struct DebugRenderTest : public ::testing::Test
{
    ovphysx_instance_handle_t h = 0;
    void SetUp() override
    {
        ovphysx_result_t created = ovphysx_create_instance(&h);
        ASSERT_EQ(created.status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(h, 0u);
    }
    void TearDown() override
    {
        if (h)
        {
            test_utils::destroy_ovstage_test_attachments(h);
            ovphysx_destroy_instance(h);
        }
    }
};

} // namespace

TEST_F(DebugRenderTest, SetParameterRejectsNoneAndOutOfRange)
{
    EXPECT_EQ(debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_NONE, 1.0f).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_COUNT, 1.0f).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    ASSERT_EQ(debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 2.5f).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, -1.0f).status,
              OVPHYSX_API_ERROR);
    EXPECT_EQ(debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES,
                                                 std::numeric_limits<float>::quiet_NaN()).status,
              OVPHYSX_API_ERROR);
    float value = 0.0f;
    ASSERT_EQ(debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &value).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value, 2.5f);
}

TEST_F(DebugRenderTest, RejectedCommitSpendsTheGroup)
{
    ASSERT_EQ(debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 2.5f).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_query(h, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
              OVPHYSX_API_SUCCESS);
    ovphysx_write_handle_t write = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_write(h, query, &OVPHYSX_NAME_DEBUG_BODY_AXES, 1, &write, nullptr)).status,
              OVPHYSX_API_SUCCESS);
    ovphysx_fetched_write_group_t written{};
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_write_next(h, write, &written)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(written.group, nullptr);
    ASSERT_NE(written.group->data.tensors, nullptr);
    ASSERT_NE(written.group->data.tensors[0].data, nullptr);
    *static_cast<float*>(written.group->data.tensors[0].data) = -1.0f;

    ovstage_cuda_sync_t sync{};
    EXPECT_EQ(test_utils::complete_task(h, ovphysx_commit_group(h, write, written.group, sync)).status, OVPHYSX_API_ERROR);
    EXPECT_NE(test_utils::last_task_error().find("spent"), std::string::npos);
    EXPECT_EQ(test_utils::complete_task(h, ovphysx_commit_group(h, write, written.group, sync)).status, OVPHYSX_API_ERROR);
    EXPECT_NE(test_utils::last_task_error().find("not live"), std::string::npos);

    float value = 0.0f;
    ASSERT_EQ(debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &value).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value, 2.5f);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_write(h, write)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_query(h, query)).status, OVPHYSX_API_SUCCESS);
}

TEST_F(DebugRenderTest, ParameterAndScaleRoundTripWithoutStage)
{
    ASSERT_EQ(debug_render_set_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, 2.5f).status,
              OVPHYSX_API_SUCCESS);
    float value = 0.0f;
    ASSERT_EQ(debug_render_get_parameter(h, OVPHYSX_DEBUG_RENDER_PARAM_BODY_AXES, &value).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(value, 2.5f);

    ASSERT_EQ(debug_render_set_scale(h, 0.25f).status, OVPHYSX_API_SUCCESS);
    float scale = 0.0f;
    ASSERT_EQ(debug_render_get_scale(h, &scale).status, OVPHYSX_API_SUCCESS);
    EXPECT_FLOAT_EQ(scale, 0.25f);
}

TEST_F(DebugRenderTest, GeometryReadIsEmptyWithoutScenes)
{
    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_query(h, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attributes[] = { OVPHYSX_NAME_DEBUG_LINE_POSITION, OVPHYSX_NAME_DEBUG_LINE_COLOR };
    ovphysx_read_handle_t read = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_read(h, query, attributes, 2, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* position = nullptr;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_read_next(h, read, &position)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(position, nullptr);
    ASSERT_NE(position->data.tensors, nullptr);
    EXPECT_EQ(position->data.tensors[0].shape[0], 0);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_group(h, read, position->read_group_id)).status, OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* color = nullptr;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_read_next(h, read, &color)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(color, nullptr);
    EXPECT_EQ(color->data.tensors[0].shape[0], 0);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_group(h, read, color->read_group_id)).status, OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* done = nullptr;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_read_next(h, read, &done)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(done, nullptr);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_read(h, read)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_query(h, query)).status, OVPHYSX_API_SUCCESS);
}

TEST_F(DebugRenderTest, UnknownWriteAttributeFails)
{
    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_query(h, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t attribute{ 0, { "notAnAttribute", sizeof("notAnAttribute") - 1 } };
    ovphysx_write_handle_t write = 0;
    EXPECT_NE(test_utils::complete_task(h, ovphysx_write(h, query, &attribute, 1, &write, nullptr)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_query(h, query)).status, OVPHYSX_API_SUCCESS);
}

TEST_F(DebugRenderTest, UnknownReadAttributeFailsWithoutShiftingColumns)
{
    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_query(h, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
              OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t unknown{ 0, { "notAnAttribute", sizeof("notAnAttribute") - 1 } };
    const ovx_string_or_token_t bad[] = { OVPHYSX_NAME_DEBUG_LINE_POSITION, unknown, OVPHYSX_NAME_DEBUG_LINE_COLOR };
    ovphysx_read_handle_t read = 0;
    EXPECT_EQ(test_utils::complete_task(h, ovphysx_read(h, query, bad, 3, &read, nullptr)).status, OVPHYSX_API_ERROR);

    const ovx_string_or_token_t attributes[] = { OVPHYSX_NAME_DEBUG_SCALE, OVPHYSX_NAME_DEBUG_ENABLED };
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_read(h, query, attributes, 2, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* scale = nullptr;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_read_next(h, read, &scale)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(scale, nullptr);
    EXPECT_EQ(scale->data.tensors[0].dtype.code, kDLFloat);
    const ovstage_read_group_t* enabled = nullptr;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_read_next(h, read, &enabled)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(enabled, nullptr);
    EXPECT_EQ(enabled->data.tensors[0].dtype.code, kDLUInt);
    EXPECT_EQ(scale->attribute, 0u);
    EXPECT_EQ(enabled->attribute, 0u);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_group(h, read, scale->read_group_id)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_group(h, read, enabled->read_group_id)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_read(h, read)).status, OVPHYSX_API_SUCCESS);

    ovx_string_or_token_t tokenOnly{};
    tokenOnly.token = 11u;
    EXPECT_EQ(test_utils::complete_task(h, ovphysx_read(h, query, &tokenOnly, 1, &read, nullptr)).status, OVPHYSX_API_ERROR);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_query(h, query)).status, OVPHYSX_API_SUCCESS);
}

TEST_F(DebugRenderTest, ScopeRoundTripAndInvalidToken)
{
    const ovx_primpath_t tokens[] = { 11u, 22u };
    ASSERT_EQ(debug_render_set_scope_tokens(h, tokens, 2u).status, OVPHYSX_API_SUCCESS);
    const ovx_primpath_t invalid = OVX_INVALID_PRIMPATH;
    EXPECT_EQ(debug_render_set_scope_tokens(h, &invalid, 1u).status, OVPHYSX_API_ERROR);
    ASSERT_EQ(debug_render_set_scope_tokens(h, nullptr, 0u).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(debug_render_set_scope_tokens(h, tokens, 4097u).status, OVPHYSX_API_ERROR);
}

TEST_F(DebugRenderTest, UnsetCullingBoxReadsEmptyAndEmptyCommitClears)
{
    ovphysx_query_handle_t query = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_query(h, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, nullptr)).status,
              OVPHYSX_API_SUCCESS);

    const ovx_string_or_token_t attribute = OVPHYSX_NAME_DEBUG_CULLING_BOX;
    ovphysx_read_handle_t read = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_read(h, query, &attribute, 1, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* group = nullptr;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_read_next(h, read, &group)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(group, nullptr);
    ASSERT_NE(group->data.tensors, nullptr);
    EXPECT_EQ(group->data.tensors[0].shape[0], 0);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_group(h, read, group->read_group_id)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_read(h, read)).status, OVPHYSX_API_SUCCESS);

    const float min_box[3] = { -1.0f, -2.0f, -3.0f };
    const float max_box[3] = { 4.0f, 5.0f, 6.0f };
    ASSERT_EQ(debug_render_set_culling_box(h, min_box, max_box).status, OVPHYSX_API_SUCCESS);

    ovphysx_write_handle_t write = 0;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_write(h, query, &attribute, 1, &write, nullptr)).status, OVPHYSX_API_SUCCESS);
    ovphysx_fetched_write_group_t written{};
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_write_next(h, write, &written)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(written.group, nullptr);
    ASSERT_NE(written.group->data.tensors, nullptr);
    ASSERT_NE(written.group->data.tensors[0].data, nullptr);
    ASSERT_EQ(written.group->data.tensors[0].shape[0], 7);
    static_cast<float*>(written.group->data.tensors[0].data)[0] = 0.0f;
    ovstage_cuda_sync_t sync{};
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_commit_group(h, write, written.group, sync)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_write(h, write)).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(test_utils::complete_task(h, ovphysx_read(h, query, &attribute, 1, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_fetch_read_next(h, read, &group)).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(group, nullptr);
    EXPECT_EQ(group->data.tensors[0].shape[0], 0);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_group(h, read, group->read_group_id)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_read(h, read)).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_query(h, query)).status, OVPHYSX_API_SUCCESS);
}

TEST_F(DebugRenderTest, SharedDictionaryIsNullWithoutStage)
{
    ovphysx_query_handle_t query = 0;
    ovstage_query_result_t summary{};
    summary.attribute_count = 1;
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_query(h, OVPHYSX_QUERY_TARGET_DEBUG_RENDER, OVPHYSX_SCOPE_ALL, &query, &summary)).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(summary.attribute_count, 0u);
    ASSERT_EQ(test_utils::complete_task(h, ovphysx_release_query(h, query)).status, OVPHYSX_API_SUCCESS);
}

TEST_F(DebugRenderTest, WritabilityClassifiesSettingsAndGeometry)
{
    ovphysx_writability_t access = OVPHYSX_WRITABILITY_UNCLASSIFIED;
    ASSERT_EQ(ovphysx_writability(OVPHYSX_QUERY_TARGET_DEBUG_RENDER, &OVPHYSX_NAME_DEBUG_BODY_AXES, &access).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(access, OVPHYSX_WRITABILITY_WRITABLE);
    ASSERT_EQ(ovphysx_writability(OVPHYSX_QUERY_TARGET_DEBUG_RENDER, &OVPHYSX_NAME_DEBUG_POINT_POSITION, &access).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(access, OVPHYSX_WRITABILITY_READ_ONLY);
}
