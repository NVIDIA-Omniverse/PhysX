// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// TODO(tensor-binding-deprecation): the articulation-metadata getters expose structural names and
// topology the session read does not, and take a binding handle with no non-binding source. Keep
// the binding until the read API grows a metadata/names path, then convert these tests.

// Tests for the articulation metadata API:
//   ovphysx_get_tensor_binding_native_device
//   ovphysx_get_articulation_metadata
//   ovphysx_articulation_get_dof_names
//   ovphysx_articulation_get_body_names
//   ovphysx_articulation_get_joint_names

/**
 * @implements REQ-CAPI-STRING-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-BINDING-STALE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 */

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
    for (uint32_t index = 0; index < out_count; ++index)
    {
        ASSERT_NE(names[index].ptr, nullptr);
        EXPECT_EQ(names[index].ptr[names[index].length], '\0');
    }
}

// Requesting zero names either succeeds with a count of 0 or returns INVALID_ARGUMENT.
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
    for (uint32_t index = 0; index < out_count; ++index)
    {
        ASSERT_NE(names[index].ptr, nullptr);
        EXPECT_EQ(names[index].ptr[names[index].length], '\0');
    }
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
    for (uint32_t index = 0; index < out_count; ++index)
    {
        ASSERT_NE(names[index].ptr, nullptr);
        EXPECT_EQ(names[index].ptr[names[index].length], '\0');
    }
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
    ASSERT_NE(path.ptr, nullptr);
    EXPECT_EQ(path.ptr[path.length], '\0');
    EXPECT_EQ(std::string(path.ptr ? path.ptr : "", path.ptr ? path.length : 0),
              "/World/articulation");
}

// ---------------------------------------------------------------------------
// Stale binding after detach/reattach (GitLab issue #30, MR !8085). The
// metadata getters must reject a binding retained across detach/reattach
// instead of returning the previous attach's shape, counts and names, the
// same way ovphysx_read_tensor_binding and ovphysx_tensor_binding_get_prim_paths do.
// ---------------------------------------------------------------------------

TEST_F(ArticulationMetadataTest, MetadataGettersRejectBindingStaleAfterReattach)
{
    LoadAndBind("tests/data/links_chain_sample.usda", "/World/articulation",
                OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32);

    // Baseline: the binding is live pre-detach, so every getter succeeds.
    ovphysx_articulation_metadata_t meta{};
    ASSERT_EQ(ovphysx_get_articulation_metadata(m_handle, m_binding, &meta).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_GT(meta.dof_count, 0);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, m_binding, &spec).status,
              OVPHYSX_API_SUCCESS);

    DLDevice device{};
    ASSERT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, m_binding, &device).status,
              OVPHYSX_API_SUCCESS);

    std::vector<ovphysx_string_t> dof_names(meta.dof_count);
    uint32_t out_count = 0;
    ASSERT_EQ(ovphysx_articulation_get_dof_names(
                  m_handle, m_binding, dof_names.data(), meta.dof_count, &out_count).status,
              OVPHYSX_API_SUCCESS);

    std::vector<ovphysx_string_t> body_names(meta.body_count);
    ASSERT_EQ(ovphysx_articulation_get_body_names(
                  m_handle, m_binding, body_names.data(), meta.body_count, &out_count).status,
              OVPHYSX_API_SUCCESS);

    std::vector<ovphysx_string_t> joint_names(meta.joint_count);
    ASSERT_EQ(ovphysx_articulation_get_joint_names(
                  m_handle, m_binding, joint_names.data(), meta.joint_count, &out_count).status,
              OVPHYSX_API_SUCCESS);

    // Detach and reattach the same USD: instance->attachHandle changes while
    // m_binding stays in the instance's tensor_bindings map (never destroyed).
    ASSERT_TRUE(destroy_ovstage_test_attachments(m_handle)) << "Failed to detach ovstage";
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/links_chain_sample.usda"))
        << "Failed to reattach ovstage USD";

    // Every metadata getter must now reject the stale binding, matching the
    // already-guarded data-path entry points below.
    EXPECT_EQ(ovphysx_get_articulation_metadata(m_handle, m_binding, &meta).status,
              OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(m_handle, m_binding, &spec).status,
              OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, m_binding, &device).status,
              OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_articulation_get_dof_names(
                  m_handle, m_binding, dof_names.data(), static_cast<uint32_t>(dof_names.size()), &out_count).status,
              OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_articulation_get_body_names(
                  m_handle, m_binding, body_names.data(), static_cast<uint32_t>(body_names.size()), &out_count).status,
              OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(ovphysx_articulation_get_joint_names(
                  m_handle, m_binding, joint_names.data(), static_cast<uint32_t>(joint_names.size()), &out_count).status,
              OVPHYSX_API_NOT_FOUND);

    // The already-guarded siblings reject the same stale handle, so the
    // getters are consistent with the data-path contract.
    ovphysx_string_t prim_path{};
    EXPECT_EQ(ovphysx_tensor_binding_get_prim_paths(m_handle, m_binding, &prim_path, 1, &out_count).status,
              OVPHYSX_API_NOT_FOUND);

    // Control: a fresh binding created after the reattach still works, so
    // the guard is attach-specific rather than a global regression.
    ovphysx_tensor_binding_handle_t fresh_binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern     = make_ovx_string("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &fresh_binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_articulation_metadata_t fresh_meta{};
    EXPECT_EQ(ovphysx_get_articulation_metadata(m_handle, fresh_binding, &fresh_meta).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(fresh_meta.dof_count, meta.dof_count);

    ovphysx_destroy_tensor_binding(m_handle, fresh_binding);
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
