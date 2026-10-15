// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-VEHICLE-001
 * @covers AC-1 AC-2
 * @maps_to TEST-CAPI-VEHICLE-001
 */

#include "global_test_environment.h"
#include "test_utilities.h"
#include "ovphysx/ovphysx_attributes.h"

#include <ovstage/ovx_path_dictionary.h>

#include <cstring>
#include <string>
#include <vector>

namespace
{
ovx_string_or_token_t attribute(const char* name)
{
    return { OVX_INVALID_TOKEN, { name, std::strlen(name) } };
}

class VehicleReadWriteTest : public PhysXTestFixture
{
protected:
    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/vehicle_read_write.usda"));
    }

    void TearDown() override
    {
        for (ovphysx_write_handle_t write : m_writes)
            EXPECT_EQ(complete(ovphysx_release_write(m_handle, write)), OVPHYSX_API_SUCCESS);
        for (ovphysx_read_handle_t read : m_reads)
            EXPECT_EQ(complete(ovphysx_release_read(m_handle, read)), OVPHYSX_API_SUCCESS);
        for (ovphysx_query_handle_t query : m_queries)
            EXPECT_EQ(complete(ovphysx_release_query(m_handle, query)), OVPHYSX_API_SUCCESS);
        PhysXTestFixture::TearDown();
    }

    ovphysx_api_status_t complete(ovphysx_enqueue_result_t task)
    {
        return test_utils::complete_task(m_handle, task).status;
    }

    ovphysx_query_handle_t query(ovphysx_query_target_t type, const char* pattern,
                                 ovstage_query_result_t* metadata = nullptr)
    {
        ovphysx_query_selection_t selection{};
        selection.pattern = { pattern, std::strlen(pattern) };
        ovphysx_query_handle_t result = 0;
        EXPECT_EQ(complete(ovphysx_query_selection(m_handle, type, &selection, &result, metadata)),
                  OVPHYSX_API_SUCCESS);
        if (result)
            m_queries.push_back(result);
        return result;
    }

    const ovstage_read_group_t* readOne(ovphysx_query_handle_t selected, const char* name)
    {
        const ovx_string_or_token_t attr = attribute(name);
        ovphysx_read_handle_t read = 0;
        EXPECT_EQ(complete(ovphysx_read(m_handle, selected, &attr, 1, &read, nullptr)), OVPHYSX_API_SUCCESS);
        if (!read)
            return nullptr;
        m_reads.push_back(read);
        const ovstage_read_group_t* group = nullptr;
        EXPECT_EQ(complete(ovphysx_fetch_read_next(m_handle, read, &group)), OVPHYSX_API_SUCCESS);
        return group;
    }

    const ovstage_map_group_t* writeOne(ovphysx_query_handle_t selected, const char* name,
                                       ovphysx_write_handle_t& write)
    {
        const ovx_string_or_token_t attr = attribute(name);
        EXPECT_EQ(complete(ovphysx_write(m_handle, selected, &attr, 1, &write, nullptr)), OVPHYSX_API_SUCCESS);
        if (!write)
            return nullptr;
        m_writes.push_back(write);
        ovphysx_fetched_write_group_t fetched{};
        EXPECT_EQ(complete(ovphysx_fetch_write_next(m_handle, write, &fetched)), OVPHYSX_API_SUCCESS);
        EXPECT_EQ(fetched.attribute_index, 0u);
        return fetched.group;
    }

    std::vector<ovphysx_query_handle_t> m_queries;
    std::vector<ovphysx_read_handle_t> m_reads;
    std::vector<ovphysx_write_handle_t> m_writes;
};

float* floatData(const DLTensor& tensor)
{
    return reinterpret_cast<float*>(static_cast<char*>(tensor.data) + tensor.byte_offset);
}
} // namespace

TEST_F(VehicleReadWriteTest, ColdFilteredWriteAndIndependentSelectors)
{
    ovstage_query_result_t metadata{};
    const ovphysx_query_handle_t vehicle = query(OVPHYSX_QUERY_TARGET_VEHICLE, "/World/Standard", &metadata);
    ASSERT_NE(vehicle, 0u);
    ASSERT_EQ(metadata.total_prim_count, 1u);
    ovphysx_write_handle_t write = 0;
    const ovstage_map_group_t* output = writeOne(vehicle, OVPHYSX_ATTR_ACCELERATOR, write);
    ASSERT_NE(output, nullptr);
    ASSERT_EQ(output->data.tensor_count, 1u);
    const DLTensor& tensor = output->data.tensors[0];
    ASSERT_EQ(tensor.device.device_type, kDLCPU);
    ASSERT_EQ(tensor.dtype.code, kDLFloat);
    ASSERT_EQ(tensor.dtype.bits, 32u);
    ASSERT_EQ(tensor.dtype.lanes, 1u);
    ASSERT_EQ(tensor.ndim, 1);
    ASSERT_EQ(tensor.shape[0], 1);
    floatData(tensor)[0] = 0.625f;
    ASSERT_EQ(complete(ovphysx_commit_group(m_handle, write, output, {})), OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const ovstage_read_group_t* control = readOne(vehicle, OVPHYSX_ATTR_ACCELERATOR);
    ASSERT_NE(control, nullptr);
    EXPECT_FLOAT_EQ(floatData(control->data.tensors[0])[0], 0.625f);

    // The same car prim selects its chassis, while a separate descendant query selects its wheels.
    const ovphysx_query_handle_t chassis = query(OVPHYSX_QUERY_TARGET_RIGID_BODY, "/World/Standard", &metadata);
    ASSERT_NE(chassis, 0u);
    EXPECT_EQ(metadata.total_prim_count, 1u);
    const ovstage_read_group_t* pose = readOne(chassis, OVPHYSX_ATTR_POSITION);
    ASSERT_NE(pose, nullptr);
    EXPECT_EQ(pose->data.tensors[0].dtype.lanes, 3u);
    ASSERT_NE(query(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, "/World/Standard/**", &metadata), 0u);
    EXPECT_EQ(metadata.total_prim_count, 4u);
    const ovphysx_query_handle_t other = query(OVPHYSX_QUERY_TARGET_VEHICLE, "/World/Basic");
    const ovstage_read_group_t* unchanged = readOne(other, OVPHYSX_ATTR_ACCELERATOR);
    ASSERT_NE(unchanged, nullptr);
    EXPECT_FLOAT_EQ(floatData(unchanged->data.tensors[0])[0], 0.0f);
}

TEST_F(VehicleReadWriteTest, TelemetryUsesFixedTypedColumns)
{
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const ovphysx_query_handle_t vehicle = query(OVPHYSX_QUERY_TARGET_VEHICLE, "/World/Standard");
    const ovphysx_query_handle_t wheels = query(OVPHYSX_QUERY_TARGET_VEHICLE_WHEEL, "/World/Standard/**");
    struct Column
    {
        const char* name;
        uint8_t code;
        uint8_t bits;
        uint16_t lanes;
        bool wheel;
    };
    const Column columns[] = {
        { OVPHYSX_ATTR_ENGINE_ROTATION_SPEED, kDLFloat, 32, 1, false },
        { OVPHYSX_ATTR_CURRENT_GEAR, kDLInt, 32, 1, false },
        { OVPHYSX_ATTR_AUTOMATIC_TRANSMISSION, kDLUInt, 8, 1, false },
        { OVPHYSX_ATTR_ROTATION_SPEED, kDLFloat, 32, 1, true },
        { OVPHYSX_ATTR_SUSPENSION_FORCE, kDLFloat, 32, 3, true },
        { OVPHYSX_ATTR_GROUND_PLANE, kDLFloat, 32, 4, true },
        { OVPHYSX_ATTR_IS_ON_GROUND, kDLUInt, 8, 1, true },
        { OVPHYSX_ATTR_GROUND_ACTOR_PATH, kDLUInt, 64, 1, true },
        { OVPHYSX_ATTR_GROUND_SHAPE_PATH, kDLUInt, 64, 1, true },
        { OVPHYSX_ATTR_GROUND_MATERIAL_PATH, kDLUInt, 64, 1, true },
    };
    for (const Column& column : columns)
    {
        SCOPED_TRACE(column.name);
        const ovstage_read_group_t* group = readOne(column.wheel ? wheels : vehicle, column.name);
        ASSERT_NE(group, nullptr);
        EXPECT_FALSE(group->is_array);
        EXPECT_EQ(group->prims.count, column.wheel ? 4u : 1u);
        ASSERT_EQ(group->data.tensor_count, 1u);
        const DLTensor& tensor = group->data.tensors[0];
        EXPECT_EQ(tensor.device.device_type, kDLCPU);
        EXPECT_EQ(tensor.ndim, 1);
        EXPECT_EQ(tensor.shape[0], column.wheel ? 4 : 1);
        EXPECT_EQ(tensor.dtype.code, column.code);
        EXPECT_EQ(tensor.dtype.bits, column.bits);
        EXPECT_EQ(tensor.dtype.lanes, column.lanes);
    }
}

TEST_F(VehicleReadWriteTest, MixedReadReportsSubtypeOmission)
{
    const ovphysx_query_handle_t all = query(OVPHYSX_QUERY_TARGET_VEHICLE, "/World/**");
    const ovx_string_or_token_t attr = attribute(OVPHYSX_ATTR_ENGINE_ROTATION_SPEED);
    ovphysx_read_handle_t read = 0;
    ovphysx_read_result_t summary{};
    ASSERT_EQ(complete(ovphysx_read(m_handle, all, &attr, 1, &read, &summary)), OVPHYSX_API_SUCCESS);
    m_reads.push_back(read);
    EXPECT_EQ(summary.total_prim_count, 4u);
    ASSERT_EQ(summary.attribute_count, 1u);
    EXPECT_NE(summary.attributes[0].omission_reasons & OVPHYSX_READ_OMISSION_UNSUPPORTED_CONFIGURATION, 0u);
    const ovstage_read_group_t* group = nullptr;
    ASSERT_EQ(complete(ovphysx_fetch_read_next(m_handle, read, &group)), OVPHYSX_API_SUCCESS);
    ASSERT_NE(group, nullptr);
    EXPECT_EQ(group->prims.count, 2u);
    ovx_string_t name{};
    ASSERT_EQ(ovx_path_dictionary_token_to_string(test_utils::attached_ovstage_dictionary(m_handle),
                                                 group->attribute, &name), OVX_OK);
    EXPECT_EQ(std::string(name.ptr, name.length), OVPHYSX_ATTR_ENGINE_ROTATION_SPEED);
}
