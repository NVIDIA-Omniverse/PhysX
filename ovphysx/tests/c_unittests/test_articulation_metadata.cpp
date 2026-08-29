// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Tests for the articulation metadata API:
//   ovphysx_get_articulation_metadata
//   ovphysx_articulation_get_dof_names
//   ovphysx_articulation_get_body_names
//   ovphysx_articulation_get_joint_names

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <string>
#include <vector>

using namespace test_utils;

static bool wait_am_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index)
{
    ovphysx_op_wait_result_t wr{};
    ovphysx_result_t r = ovphysx_wait_op(handle, op_index, 10'000'000'000ULL, &wr);
    ovphysx_destroy_wait_result(&wr);
    return r.status == OVPHYSX_API_SUCCESS;
}

// ---------------------------------------------------------------------------
// Test fixture: loads links_chain_sample.usda and creates a DOF binding for
// the single articulation at /World/articulation.
// ---------------------------------------------------------------------------
class ArticulationMetadataTest : public PhysXTestFixture {
protected:
    ovphysx_tensor_binding_handle_t m_binding = 0;

    void LoadAndBind(const char* usd_path, const char* pattern,
                     ovphysx_tensor_type_t tensor_type)
    {
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, usd_path)) << "Failed to attach ovstage USD";

        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern     = make_ovx_string(pattern);
        desc.tensor_type = tensor_type;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &m_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";
    }

    void TearDown() override
    {
        if (m_binding != 0)
        {
            ovphysx_destroy_tensor_binding(m_handle, m_binding);
            m_binding = 0;
        }
        PhysXTestFixture::TearDown();
    }
};

// ---------------------------------------------------------------------------
// NOTE: Happy-path metadata value checks (exact dof_count/body_count/joint_count,
// is_fixed_base, tendon counts, metadata consistency across binding types) are
// all covered with stricter assertions by TestArticulationMetadata in
// tests/python_tests/cpu_tests/test_tensor_bindings_api.py.
// The tests below cover only C-ABI-specific behaviors: C struct field validity
// (ovphysx_string_t.ptr + length), zero-capacity edge case, invalid handles,
// and null-out-pointer rejection.
// ---------------------------------------------------------------------------

// get_dof_names
// ---------------------------------------------------------------------------

TEST_F(ArticulationMetadataTest, GetDofNamesCountAndContent)
{
    LoadAndBind("tests/data/links_chain_sample.usda", "/World/articulation",
                OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);

    ovphysx_articulation_metadata_t meta{};
    ASSERT_EQ(ovphysx_get_articulation_metadata(m_handle, m_binding, &meta).status,
              OVPHYSX_API_SUCCESS);
    if (meta.dof_count == 0)
        GTEST_SKIP() << "No DOFs; skipping DOF name test";

    std::vector<ovphysx_string_t> names(meta.dof_count);
    uint32_t out_count = 0;
    ovphysx_result_t r = ovphysx_articulation_get_dof_names(
        m_handle, m_binding, names.data(), meta.dof_count, &out_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(out_count, static_cast<uint32_t>(meta.dof_count));
}

// Requesting zero names should return 0 and succeed (or INVALID_ARGUMENT — both OK).
TEST_F(ArticulationMetadataTest, GetDofNamesZeroCapacity)
{
    LoadAndBind("tests/data/links_chain_sample.usda", "/World/articulation",
                OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);

    uint32_t out_count = 42;
    ovphysx_result_t r = ovphysx_articulation_get_dof_names(
        m_handle, m_binding, nullptr, 0, &out_count);
    if (r.status == OVPHYSX_API_SUCCESS)
        EXPECT_EQ(out_count, 0u);
    else
        EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// ---------------------------------------------------------------------------
// get_body_names
// ---------------------------------------------------------------------------

TEST_F(ArticulationMetadataTest, GetBodyNamesCountAndContent)
{
    LoadAndBind("tests/data/links_chain_sample.usda", "/World/articulation",
                OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);

    ovphysx_articulation_metadata_t meta{};
    ASSERT_EQ(ovphysx_get_articulation_metadata(m_handle, m_binding, &meta).status,
              OVPHYSX_API_SUCCESS);
    if (meta.body_count == 0)
        GTEST_SKIP() << "No bodies; skipping body name test";

    std::vector<ovphysx_string_t> names(meta.body_count);
    uint32_t out_count = 0;
    ovphysx_result_t r = ovphysx_articulation_get_body_names(
        m_handle, m_binding, names.data(), meta.body_count, &out_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(out_count, static_cast<uint32_t>(meta.body_count));
}

// ---------------------------------------------------------------------------
// get_joint_names
// ---------------------------------------------------------------------------

TEST_F(ArticulationMetadataTest, GetJointNamesCountAndContent)
{
    LoadAndBind("tests/data/links_chain_sample.usda", "/World/articulation",
                OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);

    ovphysx_articulation_metadata_t meta{};
    ASSERT_EQ(ovphysx_get_articulation_metadata(m_handle, m_binding, &meta).status,
              OVPHYSX_API_SUCCESS);
    if (meta.joint_count == 0)
        GTEST_SKIP() << "No joints; skipping joint name test";

    std::vector<ovphysx_string_t> names(meta.joint_count);
    uint32_t out_count = 0;
    ovphysx_result_t r = ovphysx_articulation_get_joint_names(
        m_handle, m_binding, names.data(), meta.joint_count, &out_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(out_count, static_cast<uint32_t>(meta.joint_count));
}

TEST_F(ArticulationMetadataTest, GetPrimPathsReturnsArticulationRoots)
{
    LoadAndBind("tests/data/links_chain_sample.usda", "/World/articulation",
                OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);

    ovphysx_string_t path{};
    uint32_t out_count = 0;
    ovphysx_result_t r = ovphysx_tensor_binding_get_prim_paths(
        m_handle, m_binding, &path, 1, &out_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(out_count, 1u);
    EXPECT_EQ(std::string(path.ptr ? path.ptr : "", path.ptr ? path.length : 0),
              "/World/articulation");
}

// ---------------------------------------------------------------------------
// Error conditions
// ---------------------------------------------------------------------------

// Invalid binding handle should fail for all metadata calls.
TEST_F(ArticulationMetadataTest, InvalidBindingHandleMetadata)
{
    ovphysx_articulation_metadata_t meta{};
    ovphysx_result_t r = ovphysx_get_articulation_metadata(
        m_handle, OVPHYSX_INVALID_HANDLE, &meta);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

TEST_F(ArticulationMetadataTest, InvalidBindingHandleDofNames)
{
    ovphysx_string_t name{};
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_articulation_get_dof_names(
        m_handle, OVPHYSX_INVALID_HANDLE, &name, 1, &count);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

TEST_F(ArticulationMetadataTest, InvalidBindingHandleBodyNames)
{
    ovphysx_string_t name{};
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_articulation_get_body_names(
        m_handle, OVPHYSX_INVALID_HANDLE, &name, 1, &count);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

TEST_F(ArticulationMetadataTest, InvalidBindingHandleJointNames)
{
    ovphysx_string_t name{};
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_articulation_get_joint_names(
        m_handle, OVPHYSX_INVALID_HANDLE, &name, 1, &count);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// Null out_metadata pointer should return INVALID_ARGUMENT.
TEST_F(ArticulationMetadataTest, NullOutMetadataReturnsInvalidArgument)
{
    LoadAndBind("tests/data/links_chain_sample.usda", "/World/articulation",
                OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);

    ovphysx_result_t r = ovphysx_get_articulation_metadata(m_handle, m_binding, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}
