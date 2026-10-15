// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 *
 * TEST-CAPI-ASYNC-002: native receipt lifecycle and CPU session ownership/order.
 * GPU producer/consumer dependencies are exercised by the device suites.
 */

#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/experimental/Helpers.hpp>
#include <ovphysx/ovphysx_attributes.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace
{
ovx_string_or_token_t attribute(const char* name)
{
    return { 0, { name, std::strlen(name) } };
}

::testing::AssertionResult completed(ovphysx_instance_handle_t instance, ovphysx_enqueue_result_t task)
{
    if (task.status != OVPHYSX_API_SUCCESS || task.op_index == 0 || task.instance != instance)
        return ::testing::AssertionFailure() << "Submission did not return an owned task: " << task.status;
    return waitForTaskSuccess(instance, task);
}

void rejected(ovphysx_enqueue_result_t task)
{
    EXPECT_NE(task.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(task.op_index, 0u);
    EXPECT_EQ(task.instance, 0u);
}

struct OtherInstance
{
    ovphysx_instance_handle_t handle = 0;
    ~OtherInstance()
    {
        if (handle)
        {
            EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
        }
    }
};

class AsyncApiTest : public PhysXTestFixture
{
protected:
    void loadScene()
    {
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
        ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    }

    void openQuery(ovphysx_query_handle_t& query, ovphysx_query_target_t type = OVPHYSX_QUERY_TARGET_RIGID_BODY)
    {
        ASSERT_TRUE(completed(m_handle, ovphysx_query(m_handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)));
        ASSERT_NE(query, 0u);
    }

    void checkDetachedSessions()
    {
        loadScene();
        ovphysx_query_handle_t query = 0;
        openQuery(query);
        const ovx_string_or_token_t position = attribute(OVPHYSX_ATTR_POSITION);
        ovphysx_read_handle_t read = 0;
        ASSERT_TRUE(completed(m_handle, ovphysx_read(m_handle, query, &position, 1, &read, nullptr)));
        const ovstage_read_group_t* readGroup = nullptr;
        ASSERT_TRUE(completed(m_handle, ovphysx_fetch_read_next(m_handle, read, &readGroup)));
        ASSERT_NE(readGroup, nullptr);
        ASSERT_GT(readGroup->data.tensor_count, 0u);
        const DLTensor tensor = readGroup->data.tensors[0];
        ASSERT_EQ(tensor.device.device_type, kDLCPU);
        ASSERT_EQ(tensor.dtype.code, kDLFloat);
        ASSERT_EQ(tensor.dtype.bits, 32u);
        ASSERT_NE(tensor.data, nullptr);
        size_t count = tensor.dtype.lanes;
        for (int dimension = 0; dimension < tensor.ndim; ++dimension)
            count *= static_cast<size_t>(tensor.shape[dimension]);
        const float* values = reinterpret_cast<const float*>(
            static_cast<const unsigned char*>(tensor.data) + tensor.byte_offset);
        const std::vector<float> snapshot(values, values + count);
        ASSERT_TRUE(completed(m_handle, ovphysx_release_group(m_handle, read, readGroup->read_group_id)));

        ovphysx_write_handle_t write = 0;
        ASSERT_TRUE(completed(m_handle, ovphysx_write(m_handle, query, &position, 1, &write, nullptr)));
        ovphysx_fetched_write_group_t writeGroup{};
        ASSERT_TRUE(completed(m_handle, ovphysx_fetch_write_next(m_handle, write, &writeGroup)));
        ASSERT_NE(writeGroup.group, nullptr);
        ASSERT_TRUE(completed(m_handle, ovphysx_detach_ovstage(m_handle)));

        EXPECT_EQ(std::vector<float>(values, values + count), snapshot);
        ovphysx_fetched_write_group_t next{};
        rejected(ovphysx_fetch_write_next(m_handle, write, &next));
        EXPECT_EQ(next.group, nullptr);
        EXPECT_EQ(next.attribute_index, SIZE_MAX);
        rejected(ovphysx_commit_group(m_handle, write, writeGroup.group, {}));
        ASSERT_TRUE(completed(m_handle, ovphysx_release_write(m_handle, write)));
        ASSERT_TRUE(completed(m_handle, ovphysx_release_read(m_handle, read)));
        ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, query)));
    }
};
} // namespace

TEST_F(AsyncApiTest, EverySessionOperationReturnsACollectableTask)
{
    loadScene();
    ovphysx_query_handle_t query = 0;
    openQuery(query);
    const ovx_string_or_token_t position = attribute(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    ASSERT_TRUE(completed(m_handle, ovphysx_read(m_handle, query, &position, 1, &read, nullptr)));
    size_t readGroups = 0;
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        ASSERT_TRUE(completed(m_handle, ovphysx_fetch_read_next(m_handle, read, &group)));
        if (!group)
            break;
        ASSERT_LT(++readGroups, 100u);
        ASSERT_GT(group->data.tensor_count, 0u);
        const DLTensor tensor = group->data.tensors[0];
        ASSERT_NE(tensor.shape, nullptr);
        const int64_t rows = tensor.shape[0];
        ASSERT_TRUE(completed(m_handle, ovphysx_release_group(m_handle, read, group->read_group_id)));
        EXPECT_EQ(tensor.shape[0], rows); // Numeric descriptors belong to the session.
    }
    EXPECT_GT(readGroups, 0u);
    ASSERT_TRUE(completed(m_handle, ovphysx_release_read(m_handle, read)));

    const ovx_string_or_token_t velocity = attribute(OVPHYSX_ATTR_LINEAR_VELOCITY);
    ovphysx_write_handle_t write = 0;
    ASSERT_TRUE(completed(m_handle, ovphysx_write(m_handle, query, &velocity, 1, &write, nullptr)));
    size_t writeGroups = 0;
    for (;;)
    {
        ovphysx_fetched_write_group_t fetched{};
        ASSERT_TRUE(completed(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)));
        if (!fetched.group)
            break;
        const ovstage_map_group_t* group = fetched.group;
        ASSERT_LT(++writeGroups, 100u);
        for (size_t index = 0; index < group->data.tensor_count; ++index)
        {
            const DLTensor& tensor = group->data.tensors[index];
            ASSERT_EQ(tensor.device.device_type, kDLCPU);
            ASSERT_EQ(tensor.dtype.code, kDLFloat);
            ASSERT_EQ(tensor.dtype.bits, 32u);
            size_t count = tensor.dtype.lanes;
            for (int dimension = 0; dimension < tensor.ndim; ++dimension)
                count *= static_cast<size_t>(tensor.shape[dimension]);
            float* values = reinterpret_cast<float*>(static_cast<unsigned char*>(tensor.data) + tensor.byte_offset);
            std::fill(values, values + count, 0.0f);
        }
        ASSERT_TRUE(completed(m_handle, ovphysx_commit_group(m_handle, write, group, {})));
    }
    EXPECT_GT(writeGroups, 0u);
    ASSERT_TRUE(completed(m_handle, ovphysx_release_write(m_handle, write)));
    ovphysx_query_handle_t articulationQuery = 0;
    openQuery(articulationQuery, OVPHYSX_QUERY_TARGET_ARTICULATION);
    ASSERT_TRUE(completed(
        m_handle,
        ovphysx_update_articulations_kinematic(
            m_handle,
            articulationQuery,
            OVPHYSX_ARTICULATION_KINEMATIC_POSITION |
                OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, articulationQuery)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, query)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, query)));
}

TEST_F(AsyncApiTest, RejectionHasNoTaskAndAnAcceptedFailureSurvivesOtherWaits)
{
    loadScene();
    rejected(ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, nullptr, nullptr));
    rejected(ovphysx_fetch_read_next(m_handle, 0, nullptr));
    rejected(ovphysx_fetch_write_next(m_handle, 0, nullptr));
    rejected(ovphysx_commit_group(m_handle, 0, nullptr, {}));

    ovphysx_query_handle_t query = 0;
    openQuery(query);
    rejected(ovphysx_update_articulations_kinematic(m_handle, query, 1u << 31));
    rejected(ovphysx_update_articulations_kinematic(
        m_handle, query, OVPHYSX_ARTICULATION_KINEMATIC_POSITION));

    const ovx_string_or_token_t unsupported = attribute("notAnAttribute");
    ovphysx_write_handle_t invalidWrite = 99;
    const ovphysx_enqueue_result_t failed = ovphysx_write(m_handle, query, &unsupported, 1, &invalidWrite, nullptr);
    ASSERT_EQ(failed.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(failed.op_index, 0u);
    EXPECT_EQ(invalidWrite, 0u);

    const ovx_string_or_token_t position = attribute(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    ASSERT_TRUE(completed(m_handle, ovphysx_read(m_handle, query, &position, 1, &read, nullptr)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_read(m_handle, read)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, query)));

    ovphysx::physx::WaitResult result;
    EXPECT_EQ(result.waitTask(m_handle, failed), OVPHYSX_API_ERROR);
    ASSERT_EQ(result.errorCount(), 1u);
    EXPECT_EQ(result.errorTaskAt(0).op_index, failed.op_index);
    const ovphysx_string_t error = ovphysx_get_last_task_error(m_handle, result.errorTaskAt(0));
    const std::string message(error.ptr ? error.ptr : "", error.length);
    EXPECT_NE(message.find("notAnAttribute"), std::string::npos) << message;
    ovphysx::physx::WaitResult repeated;
    EXPECT_EQ(repeated.waitTask(m_handle, failed), OVPHYSX_API_NOT_FOUND);
}

TEST_F(AsyncApiTest, ForeignResourcesAreRejectedWithoutEndingOwnerSessions)
{
    loadScene();
    OtherInstance second;
    ASSERT_EQ(ovphysx_create_instance(&second.handle).status, OVPHYSX_API_SUCCESS);
    const ovphysx_instance_handle_t other = second.handle;
    ovphysx_query_handle_t query = 0;
    openQuery(query);
    const ovx_string_or_token_t position = attribute(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t read = 0;
    ovphysx_write_handle_t write = 0;
    ASSERT_TRUE(completed(m_handle, ovphysx_read(m_handle, query, &position, 1, &read, nullptr)));
    ASSERT_TRUE(completed(m_handle, ovphysx_write(m_handle, query, &position, 1, &write, nullptr)));

    rejected(ovphysx_release_query(other, query));
    rejected(ovphysx_release_read(other, read));
    rejected(ovphysx_release_write(other, write));
    const ovstage_read_group_t* readGroup = nullptr;
    ovphysx_fetched_write_group_t writeGroup{};
    rejected(ovphysx_fetch_read_next(other, read, &readGroup));
    rejected(ovphysx_fetch_write_next(other, write, &writeGroup));
    EXPECT_EQ(readGroup, nullptr);
    EXPECT_EQ(writeGroup.group, nullptr);
    EXPECT_EQ(writeGroup.attribute_index, SIZE_MAX);

    ASSERT_TRUE(completed(m_handle, ovphysx_fetch_read_next(m_handle, read, &readGroup)));
    ASSERT_NE(readGroup, nullptr);
    rejected(ovphysx_release_group(other, read, readGroup->read_group_id));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_group(m_handle, read, readGroup->read_group_id)));
    ASSERT_TRUE(completed(m_handle, ovphysx_fetch_write_next(m_handle, write, &writeGroup)));
    ASSERT_NE(writeGroup.group, nullptr);
    rejected(ovphysx_commit_group(other, write, writeGroup.group, {}));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_write(m_handle, write)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_read(m_handle, read)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, query)));
}

TEST_F(AsyncApiTest, InlineOperationsRespectStepsStartedAfterSessionCreation)
{
    loadScene();
    ovphysx_query_handle_t query = 0;
    openQuery(query);
    const ovx_string_or_token_t position = attribute(OVPHYSX_ATTR_POSITION);
    const ovphysx_enqueue_result_t stepBeforeRead = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(stepBeforeRead.status, OVPHYSX_API_SUCCESS);
    ovphysx_read_handle_t read = 0;
    ASSERT_TRUE(completed(m_handle, ovphysx_read(m_handle, query, &position, 1, &read, nullptr)));
    ASSERT_TRUE(completed(m_handle, stepBeforeRead));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_read(m_handle, read)));

    const ovx_string_or_token_t velocity = attribute(OVPHYSX_ATTR_LINEAR_VELOCITY);
    ovphysx_write_handle_t write = 0;
    ASSERT_TRUE(completed(m_handle, ovphysx_write(m_handle, query, &velocity, 1, &write, nullptr)));
    ovphysx_fetched_write_group_t fetched{};
    ASSERT_TRUE(completed(m_handle, ovphysx_fetch_write_next(m_handle, write, &fetched)));
    ASSERT_NE(fetched.group, nullptr);
    const ovstage_map_group_t* group = fetched.group;
    for (size_t index = 0; index < group->data.tensor_count; ++index)
    {
        const DLTensor& tensor = group->data.tensors[index];
        ASSERT_EQ(tensor.device.device_type, kDLCPU);
        ASSERT_EQ(tensor.dtype.code, kDLFloat);
        ASSERT_EQ(tensor.dtype.bits, 32u);
        size_t count = tensor.dtype.lanes;
        for (int dimension = 0; dimension < tensor.ndim; ++dimension)
            count *= static_cast<size_t>(tensor.shape[dimension]);
        float* values = reinterpret_cast<float*>(static_cast<unsigned char*>(tensor.data) + tensor.byte_offset);
        std::fill(values, values + count, 0.0f);
    }
    const ovphysx_enqueue_result_t stepBeforeCommit = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(stepBeforeCommit.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(completed(m_handle, ovphysx_commit_group(m_handle, write, group, {})));
    ASSERT_TRUE(completed(m_handle, stepBeforeCommit));
    const ovphysx_enqueue_result_t spent = ovphysx_commit_group(m_handle, write, group, {});
    ASSERT_EQ(spent.status, OVPHYSX_API_SUCCESS);
    ovphysx::physx::WaitResult failed;
    EXPECT_EQ(failed.waitTask(m_handle, spent), OVPHYSX_API_ERROR);
    ASSERT_EQ(failed.errorCount(), 1u);
    ASSERT_TRUE(completed(m_handle, ovphysx_release_write(m_handle, write)));
    ovphysx_query_handle_t articulationQuery = 0;
    openQuery(articulationQuery, OVPHYSX_QUERY_TARGET_ARTICULATION);
    ASSERT_TRUE(completed(
        m_handle,
        ovphysx_update_articulations_kinematic(
            m_handle,
            articulationQuery,
            OVPHYSX_ARTICULATION_KINEMATIC_POSITION |
                OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY)));
    ASSERT_TRUE(completed(m_handle, ovphysx_step(m_handle, 1.0f / 60.0f)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, articulationQuery)));
    ASSERT_TRUE(completed(m_handle, ovphysx_release_query(m_handle, query)));
}

TEST_F(AsyncApiTest, DetachRetiresWritesAndPreservesBorrowedReadStorage)
{
    checkDetachedSessions();
}
