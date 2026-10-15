// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-SESSION-001
// @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-7 AC-8 AC-9

#include "global_test_environment.h"
#include "test_utilities.h"
#include "ovphysx/ovphysx_attributes.h"

#include <ovx/path_dictionary/path_dictionary_utils.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
ovx_string_or_token_t sessionAttribute(const char* name)
{
    return { OVX_INVALID_TOKEN, { name, std::strlen(name) } };
}

size_t scalarCount(const DLTensor& tensor)
{
    size_t count = tensor.dtype.lanes;
    for (int dimension = 0; dimension < tensor.ndim; ++dimension)
        count *= static_cast<size_t>(tensor.shape[dimension]);
    return count;
}
} // namespace

class SessionResultsTest : public PhysXTestFixture
{
protected:
    void attachAndStep()
    {
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(
            m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
        ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    }

    path_dictionary_instance_t* dictionary()
    {
        std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
        const std::vector<test_utils::OvstageTestAttachment>& attachments =
            test_utils::ovstage_test_attachments().at(m_handle);
        ovstage_instance_t* stage = attachments.back().stage;
        return stage->vtable->get_path_dictionary(stage->context);
    }
};

TEST_F(SessionResultsTest, ReadMetadataPreservesRequestIdentityAndQueryIndependence)
{
    attachAndStep();
    ovphysx_query_handle_t query = 0;
    ovstage_query_result_t discovery{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, &discovery)));
    ASSERT_GT(discovery.total_prim_count, 0u);
    EXPECT_EQ(discovery.query_result_id, 0u);
    EXPECT_EQ(discovery.all_handle, 0u);
    const size_t membershipCount = discovery.total_prim_count;

    const ovx_string_or_token_t attributes[] = {
        sessionAttribute(OVPHYSX_ATTR_POSITION), sessionAttribute("unknownOne"),
        sessionAttribute("unknownTwo"), sessionAttribute(OVPHYSX_ATTR_ORIENTATION)
    };
    ovphysx_read_handle_t read = 0;
    ovphysx_read_result_t summary{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_read(m_handle, query, attributes, 4, &read, &summary)));
    ASSERT_EQ(summary.attribute_count, 4u);
    ASSERT_NE(summary.attributes, nullptr);
    EXPECT_EQ(summary.total_prim_count, membershipCount);
    EXPECT_GT(summary.attributes[0].group_count, 0u);
    EXPECT_GT(summary.attributes[3].group_count, 0u);
    EXPECT_EQ(summary.attributes[0].omission_reasons, 0u);
    EXPECT_EQ(summary.attributes[3].omission_reasons, 0u);
    for (size_t index : { 1u, 2u })
    {
        EXPECT_EQ(summary.attributes[index].attribute, OVX_INVALID_TOKEN);
        EXPECT_EQ(summary.attributes[index].group_count, 0u);
        EXPECT_EQ(summary.attributes[index].omission_reasons, OVPHYSX_READ_OMISSION_UNSUPPORTED_ATTRIBUTE);
    }
    const ovx_token_t position = summary.attributes[0].attribute;
    const ovx_token_t orientation = summary.attributes[3].attribute;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));

    std::array<size_t, 2> counts{};
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
        if (!group)
            break;
        ASSERT_TRUE(group->attribute == position || group->attribute == orientation);
        ++counts[group->attribute == position ? 0 : 1];
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_group(m_handle, read, group->read_group_id)));
    }
    EXPECT_EQ(counts[0], summary.attributes[0].group_count);
    EXPECT_EQ(counts[1], summary.attributes[3].group_count);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
}

TEST_F(SessionResultsTest, CanonicalDuplicatesAndInvalidTokensFailWithoutPublishedOutputs)
{
    attachAndStep();
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    const ovx_string_t positionName{ OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 };
    ovx_token_t position = OVX_INVALID_TOKEN;
    ASSERT_EQ(path_dictionary_create_tokens_from_strings(dictionary(), &positionName, 1, &position).status,
              OVX_API_SUCCESS);
    const ovx_string_or_token_t tokenWins{ position, { nullptr, 99 } };
    ovphysx_read_handle_t tokenRead = 0;
    ovphysx_read_result_t tokenReadSummary{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_read(
        m_handle, query, &tokenWins, 1, &tokenRead, &tokenReadSummary)));
    ASSERT_EQ(tokenReadSummary.attribute_count, 1u);
    EXPECT_EQ(tokenReadSummary.attributes[0].attribute, position);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, tokenRead)));
    ovphysx_write_handle_t tokenWrite = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(
        m_handle, query, &tokenWins, 1, &tokenWrite, nullptr)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, tokenWrite)));
    const ovx_string_or_token_t duplicate[] = {
        sessionAttribute(OVPHYSX_ATTR_POSITION), { position, { "ignored", 7 } }
    };
    ovphysx_read_handle_t read = 999;
    ovphysx_read_result_t readSummary{ 999, 999, nullptr };
    ovphysx_enqueue_result_t submitted = ovphysx_read(m_handle, query, duplicate, 2, &read, &readSummary);
    ASSERT_EQ(submitted.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(test_utils::complete_task(m_handle, submitted).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(read, 0u);
    EXPECT_EQ(readSummary.attribute_count, 0u);
    EXPECT_EQ(readSummary.total_prim_count, 0u);
    EXPECT_EQ(readSummary.attributes, nullptr);

    ovphysx_write_handle_t write = 999;
    ovphysx_write_result_t writeSummary{ 999, 999, nullptr };
    submitted = ovphysx_write(m_handle, query, duplicate, 2, &write, &writeSummary);
    ASSERT_EQ(submitted.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(test_utils::complete_task(m_handle, submitted).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(write, 0u);
    EXPECT_EQ(writeSummary.attribute_count, 0u);
    EXPECT_EQ(writeSummary.attributes, nullptr);

    const ovx_string_or_token_t invalidToken{ std::numeric_limits<ovx_token_t>::max(), positionName };
    submitted = ovphysx_read(m_handle, query, &invalidToken, 1, &read, &readSummary);
    ASSERT_EQ(submitted.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(test_utils::complete_task(m_handle, submitted).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(read, 0u);
    EXPECT_EQ(readSummary.attributes, nullptr);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

TEST_F(SessionResultsTest, MultiAttributeWriteUsesOriginalRequestIndexAndOriginalGroupPointer)
{
    attachAndStep();
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    const ovx_string_or_token_t attributes[] = {
        sessionAttribute(OVPHYSX_ATTR_LINEAR_VELOCITY), sessionAttribute(OVPHYSX_ATTR_ANGULAR_VELOCITY)
    };
    const std::array<float, 2> values{ 3.0f, 7.0f };
    ovphysx_write_handle_t write = 0;
    ovphysx_write_result_t summary{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(m_handle, query, attributes, 2, &write, &summary)));
    ASSERT_EQ(summary.attribute_count, 2u);
    const std::array<ovx_token_t, 2> tokens{ summary.attributes[0].attribute, summary.attributes[1].attribute };
    EXPECT_NE(tokens[0], tokens[1]);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));

    std::vector<ovphysx_fetched_write_group_t> fetched;
    std::array<size_t, 2> counts{};
    for (;;)
    {
        ovphysx_fetched_write_group_t result{};
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_write_next(m_handle, write, &result)));
        if (!result.group)
        {
            EXPECT_EQ(result.attribute_index, SIZE_MAX);
            break;
        }
        ASSERT_LT(result.attribute_index, 2u);
        ++counts[result.attribute_index];
        ASSERT_EQ(result.group->data.tensor_count, 1u);
        const DLTensor& tensor = result.group->data.tensors[0];
        ASSERT_EQ(tensor.device.device_type, kDLCPU);
        ASSERT_EQ(tensor.dtype.code, kDLFloat);
        ASSERT_EQ(tensor.dtype.bits, 32u);
        float* data = reinterpret_cast<float*>(static_cast<unsigned char*>(tensor.data) + tensor.byte_offset);
        std::fill(data, data + scalarCount(tensor), values[result.attribute_index]);
        fetched.push_back(result);
    }
    for (size_t index = 0; index < 2; ++index)
    {
        EXPECT_GT(counts[index], 0u);
        EXPECT_EQ(counts[index], summary.attributes[index].group_count);
    }
    // Keep earlier pointers through later fetches, then publish in reverse order.
    for (std::vector<ovphysx_fetched_write_group_t>::reverse_iterator it = fetched.rbegin(); it != fetched.rend(); ++it)
    {
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_commit_group(m_handle, write, it->group, {})));
        if (it == fetched.rbegin())
        {
            const ovphysx_enqueue_result_t repeated = ovphysx_commit_group(m_handle, write, it->group, {});
            ASSERT_EQ(repeated.status, OVPHYSX_API_SUCCESS);
            EXPECT_NE(test_utils::complete_task(m_handle, repeated).status, OVPHYSX_API_SUCCESS);
        }
    }
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));

    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    ovphysx_read_handle_t read = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_read(m_handle, query, attributes, 2, &read, nullptr)));
    std::array<size_t, 2> scalarCounts{};
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
        if (!group)
            break;
        ASSERT_TRUE(group->attribute == tokens[0] || group->attribute == tokens[1]);
        const size_t index = group->attribute == tokens[0] ? 0 : 1;
        for (size_t tensorIndex = 0; tensorIndex < group->data.tensor_count; ++tensorIndex)
        {
            const DLTensor& tensor = group->data.tensors[tensorIndex];
            ASSERT_EQ(tensor.device.device_type, kDLCPU);
            const float* data = reinterpret_cast<const float*>(
                static_cast<const unsigned char*>(tensor.data) + tensor.byte_offset);
            const size_t count = scalarCount(tensor);
            scalarCounts[index] += count;
            for (size_t offset = 0; offset < count; ++offset)
                EXPECT_FLOAT_EQ(data[offset], values[index]);
        }
        ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_group(m_handle, read, group->read_group_id)));
    }
    EXPECT_GT(scalarCounts[0], 0u);
    EXPECT_EQ(scalarCounts[0], scalarCounts[1]);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

TEST_F(SessionResultsTest, InvalidMiddleWriteAttributePublishesNothing)
{
    attachAndStep();
    ovphysx_query_handle_t query = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query, nullptr)));
    const ovx_string_or_token_t attributes[] = {
        sessionAttribute(OVPHYSX_ATTR_POSITION), sessionAttribute("unknownWrite"),
        sessionAttribute(OVPHYSX_ATTR_LINEAR_VELOCITY)
    };
    ovphysx_write_handle_t write = 999;
    ovphysx_write_result_t summary{ 999, 999, nullptr };
    const ovphysx_enqueue_result_t submitted = ovphysx_write(m_handle, query, attributes, 3, &write, &summary);
    ASSERT_EQ(submitted.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(test_utils::complete_task(m_handle, submitted).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(write, 0u);
    EXPECT_EQ(summary.total_prim_count, 0u);
    EXPECT_EQ(summary.attribute_count, 0u);
    EXPECT_EQ(summary.attributes, nullptr);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
}

TEST_F(SessionResultsTest, SelectedEmptyQueryReturnsValidEmptySessionSummaries)
{
    attachAndStep();
    ovphysx_query_selection_t selection{};
    selection.pattern = OVPHYSX_LITERAL("/NeverMatchesAnyPhysicsObject*");
    ovphysx_query_handle_t query = 0;
    ovstage_query_result_t discovery{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query_selection(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, &selection, &query, &discovery)));
    EXPECT_EQ(discovery.total_prim_count, 0u);
    EXPECT_EQ(discovery.query_result_id, 0u);
    EXPECT_EQ(discovery.all_handle, 0u);
    const ovx_string_or_token_t attribute = sessionAttribute(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    ovphysx_read_result_t readSummary{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_read(m_handle, query, &attribute, 1, &read, &readSummary)));
    ASSERT_EQ(readSummary.attribute_count, 1u);
    EXPECT_EQ(readSummary.total_prim_count, 0u);
    EXPECT_EQ(readSummary.attributes[0].group_count, 0u);
    EXPECT_EQ(readSummary.attributes[0].omission_reasons, 0u);
    ovphysx_write_handle_t write = 0;
    ovphysx_write_result_t writeSummary{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(m_handle, query, &attribute, 1, &write, &writeSummary)));
    ASSERT_EQ(writeSummary.attribute_count, 1u);
    EXPECT_EQ(writeSummary.total_prim_count, 0u);
    EXPECT_EQ(writeSummary.attributes[0].group_count, 0u);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, query)));
    ovphysx_fetched_write_group_t fetched{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)));
    EXPECT_EQ(fetched.group, nullptr);
    EXPECT_EQ(fetched.attribute_index, SIZE_MAX);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, write)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, read)));
}

TEST_F(SessionResultsTest, ReattachingEqualPathsDoesNotReviveOldSessions)
{
    attachAndStep();
    ovphysx_query_handle_t oldQuery = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &oldQuery, nullptr)));
    const ovx_string_or_token_t attribute = sessionAttribute(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t oldRead = 0;
    ovphysx_write_handle_t oldWrite = 0;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_read(
        m_handle, oldQuery, &attribute, 1, &oldRead, nullptr)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_write(
        m_handle, oldQuery, &attribute, 1, &oldWrite, nullptr)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_detach_ovstage(m_handle)));
    attachAndStep();

    const auto fails = [this](ovphysx_enqueue_result_t receipt) {
        return receipt.status != OVPHYSX_API_SUCCESS ||
               test_utils::complete_task(m_handle, receipt).status != OVPHYSX_API_SUCCESS;
    };
    ovphysx_read_handle_t rejectedRead = 999;
    ovphysx_read_result_t rejectedSummary{ 999, 999, nullptr };
    EXPECT_TRUE(fails(ovphysx_read(m_handle, oldQuery, &attribute, 1, &rejectedRead, &rejectedSummary)));
    EXPECT_EQ(rejectedRead, 0u);
    EXPECT_EQ(rejectedSummary.attribute_count, 0u);
    EXPECT_EQ(rejectedSummary.attributes, nullptr);
    const ovstage_read_group_t* oldGroup = nullptr;
    EXPECT_TRUE(fails(ovphysx_fetch_read_next(m_handle, oldRead, &oldGroup)));
    EXPECT_EQ(oldGroup, nullptr);
    ovphysx_fetched_write_group_t oldWriteGroup{};
    EXPECT_TRUE(fails(ovphysx_fetch_write_next(m_handle, oldWrite, &oldWriteGroup)));
    EXPECT_EQ(oldWriteGroup.group, nullptr);
    EXPECT_EQ(oldWriteGroup.attribute_index, SIZE_MAX);

    ovphysx_query_handle_t newQuery = 0;
    ovstage_query_result_t discovery{};
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_query(
        m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &newQuery, &discovery)));
    EXPECT_NE(newQuery, oldQuery);
    EXPECT_GT(discovery.total_prim_count, 0u);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_read(m_handle, oldRead)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_write(m_handle, oldWrite)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, oldQuery)));
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_release_query(m_handle, newQuery)));
}
