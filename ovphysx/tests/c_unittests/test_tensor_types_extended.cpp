// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// C-ABI coverage for the extended tensor type set.
//
// NOTE: Exact shape checks, read/write round-trips, masked/indexed writes,
// read-only and write-only enforcement for most types, and metadata
// consistency are all tested more thoroughly by the Python suite:
//   tests/python_tests/cpu_tests/test_tensor_bindings_api.py
//   (TestDofProperties, TestBodyProperties, TestDynamicsTensors,
//    TestFixedTendon, TestSpatialTendon, TestLinkWrench, TestRigidBodyProperties)
//
// This file keeps focused C-ABI tests:
//
//     Representative creation and metadata checks exercise the create ->
//     get_spec -> destroy path plus rigid-body row-path lookup. Broader enum
//     coverage is delegated to the Python suite.
//
//     C-ABI argument validation covers invalid instance handles, null pointers,
//     and out-of-range tensor type enums without crashing.
//
//     Access-mode checks cover read-only types not explicitly tested for write
//     rejection by the Python suite:
//       - ARTICULATION_LINK_POSE_F32          (read-only)
//       - ARTICULATION_BODY_INV_MASS_F32      (read-only)
//       - ARTICULATION_BODY_INV_INERTIA_F32   (read-only)

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <vector>
#include <cstdint>

using namespace test_utils;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool wait_ext_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index)
{
    ovphysx_op_wait_result_t wr{};
    ovphysx_result_t r = ovphysx_wait_op(handle, op_index, 10'000'000'000ULL, &wr);
    ovphysx_destroy_wait_result(&wr);
    return r.status == OVPHYSX_API_SUCCESS;
}

static bool load_scene(ovphysx_handle_t handle, const char* path)
{
    return attach_usd_with_ovstage(handle, path);
}

// Try to create a binding + get its spec.  Returns the binding handle (caller
// must destroy), or 0 on failure (test emits a non-fatal failure).
static ovphysx_tensor_binding_handle_t try_create(
    ovphysx_handle_t handle,
    const char* pattern,
    ovphysx_tensor_type_t type,
    const char* type_name)
{
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern     = make_ovx_string(pattern);
    desc.tensor_type = type;
    ovphysx_tensor_binding_handle_t binding = 0;

    ovphysx_result_t r = ovphysx_create_tensor_binding(handle, &desc, &binding);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS)
        << "create_tensor_binding failed for " << type_name;
    if (r.status != OVPHYSX_API_SUCCESS) return 0;

    ovphysx_tensor_spec_t spec{};
    r = ovphysx_get_tensor_binding_spec(handle, binding, &spec);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS)
        << "get_tensor_binding_spec failed for " << type_name;
    if (r.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_destroy_tensor_binding(handle, binding);
        return 0;
    }
    return binding;
}

// Attempt a write-zero into a binding; return the status (caller asserts).
static ovphysx_api_status_t try_write(ovphysx_handle_t handle,
                                      ovphysx_tensor_binding_handle_t binding,
                                      const ovphysx_tensor_spec_t& spec)
{
    int64_t total = 1;
    for (int d = 0; d < spec.ndim; ++d) total *= spec.shape[d];
    if (total == 0) return OVPHYSX_API_SUCCESS; // vacuously fine

    std::vector<float> zeros(total, 0.0f);
    int64_t shape[4] = {spec.shape[0], spec.shape[1], spec.shape[2], spec.shape[3]};
    DLTensor src{};
    src.data        = zeros.data();
    src.device      = {kDLCPU, 0};
    src.ndim        = spec.ndim;
    src.dtype       = {kDLFloat, 32, 1};
    src.shape       = shape;
    src.strides     = nullptr;
    src.byte_offset = 0;

    ovphysx_result_t r = ovphysx_write_tensor_binding(handle, binding, &src, nullptr);
    return static_cast<ovphysx_api_status_t>(r.status);
}

// ============================================================================
// Fixture aliases
// ============================================================================

class TensorTypesExtCpuTest  : public PhysXTestFixture {};
class TensorTypesExtBodyTest : public PhysXTestFixture {};

// ============================================================================
// REPRESENTATIVE CREATION AND METADATA CHECKS
// ============================================================================

TEST_F(TensorTypesExtCpuTest, RigidBodyMassCreation)
{
    ASSERT_TRUE(load_scene(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));

    auto b = try_create(m_handle, "/World/Cube*",
                        OVPHYSX_TENSOR_RIGID_BODY_MASS_F32, "RIGID_BODY_MASS_F32");
    ASSERT_NE(b, 0u) << "create_tensor_binding must succeed for RIGID_BODY_MASS_F32";

    ovphysx_tensor_spec_t spec{};
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(m_handle, b, &spec).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, b);
}

TEST_F(TensorTypesExtCpuTest, RigidBodyPrimPaths)
{
    ASSERT_TRUE(load_scene(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));

    auto b = try_create(m_handle, "/World/Cube*",
                        OVPHYSX_TENSOR_RIGID_BODY_POSE_F32, "RIGID_BODY_POSE_F32");
    ASSERT_NE(b, 0u) << "create_tensor_binding must succeed for RIGID_BODY_POSE_F32";

    ovphysx_string_t paths[16]{};
    uint32_t out_count = 0;
    ovphysx_result_t r = ovphysx_tensor_binding_get_prim_paths(
        m_handle, b, paths, 16, &out_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(out_count, 0u);
    EXPECT_NE(paths[0].ptr, nullptr);
    EXPECT_GT(paths[0].length, 0u);

    ovphysx_destroy_tensor_binding(m_handle, b);
}

TEST_F(TensorTypesExtCpuTest, ArticulationBodyMassCreation)
{
    ASSERT_TRUE(load_scene(m_handle, "tests/data/links_chain_sample.usda"));

    auto b = try_create(m_handle, "/World/articulation",
                        OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32,
                        "ARTICULATION_BODY_MASS_F32");
    ASSERT_NE(b, 0u) << "create_tensor_binding must succeed for ARTICULATION_BODY_MASS_F32";

    ovphysx_tensor_spec_t spec{};
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(m_handle, b, &spec).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, b);
}

// ============================================================================
// C-ABI ARGUMENT VALIDATION
// Null / invalid arguments must be rejected without crashing.
// ============================================================================

// Invalid instance handle must return non-SUCCESS.
TEST_F(TensorTypesExtCpuTest, CreateBindingInvalidInstanceHandle)
{
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern     = make_ovx_string("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_MASS_F32;
    ovphysx_tensor_binding_handle_t b = 0;
    ovphysx_result_t r = ovphysx_create_tensor_binding(OVPHYSX_INVALID_HANDLE, &desc, &b);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "Invalid instance handle must be rejected";
    if (b) ovphysx_destroy_tensor_binding(m_handle, b);
}

// Null descriptor (null path) must return INVALID_ARGUMENT.
TEST_F(TensorTypesExtCpuTest, CreateBindingNullDescriptor)
{
    ovphysx_tensor_binding_handle_t b = 0;
    ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, nullptr, &b);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "Null descriptor must return INVALID_ARGUMENT";
}

// Null out_binding pointer must return INVALID_ARGUMENT.
TEST_F(TensorTypesExtCpuTest, CreateBindingNullOutBinding)
{
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern     = make_ovx_string("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_MASS_F32;
    ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "Null out_binding pointer must return INVALID_ARGUMENT";
}

// Null out_spec pointer passed to get_tensor_binding_spec must return
// INVALID_ARGUMENT.
TEST_F(TensorTypesExtCpuTest, GetSpecNullOutSpec)
{
    ASSERT_TRUE(load_scene(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));

    auto b = try_create(m_handle, "/World/Cube*",
                        OVPHYSX_TENSOR_RIGID_BODY_MASS_F32, "RIGID_BODY_MASS_F32");
    ASSERT_NE(b, 0u) << "Precondition: binding must be created successfully";

    ovphysx_result_t r = ovphysx_get_tensor_binding_spec(m_handle, b, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "Null out_spec pointer must return INVALID_ARGUMENT";

    ovphysx_destroy_tensor_binding(m_handle, b);
}

// Out-of-range tensor type enum must be rejected.
TEST_F(TensorTypesExtCpuTest, CreateBindingInvalidTensorType)
{
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern     = make_ovx_string("/World/Cube*");
    desc.tensor_type = static_cast<ovphysx_tensor_type_t>(0x7FFF);
    ovphysx_tensor_binding_handle_t b = 0;
    ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &b);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS)
        << "Out-of-range tensor type must be rejected";
    if (b) ovphysx_destroy_tensor_binding(m_handle, b);
}

// ============================================================================
// ACCESS-MODE ENFORCEMENT
// Python explicitly tests LINK_ACCELERATION, LINK_WRENCH, and all dynamics
// types. The types below are read-only but not explicitly tested for write
// rejection in the Python suite.
// ============================================================================

// LINK_POSE is read-only: a write attempt must return non-SUCCESS.
TEST_F(TensorTypesExtCpuTest, ArticulationLinkPoseWriteRejected)
{
    ASSERT_TRUE(load_scene(m_handle, "tests/data/links_chain_sample.usda"));

    ovphysx_tensor_spec_t spec{};
    auto b = try_create(m_handle, "/World/articulation",
                        OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32, "LINK_POSE");
    if (!b) GTEST_SKIP() << "Binding creation failed";
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, b, &spec).status,
              OVPHYSX_API_SUCCESS);

    if (spec.shape[0] > 0) {
        EXPECT_NE(try_write(m_handle, b, spec), OVPHYSX_API_SUCCESS)
            << "LINK_POSE is read-only; write must be rejected";
    }

    ovphysx_destroy_tensor_binding(m_handle, b);
}

// BODY_INV_MASS is read-only: a write attempt must return non-SUCCESS.
TEST_F(TensorTypesExtBodyTest, BodyInvMassWriteRejected)
{
    ASSERT_TRUE(load_scene(m_handle, "tests/data/links_chain_sample.usda"));

    ovphysx_tensor_spec_t spec{};
    auto b = try_create(m_handle, "/World/articulation",
                        OVPHYSX_TENSOR_ARTICULATION_BODY_INV_MASS_F32, "BODY_INV_MASS");
    if (!b) GTEST_SKIP() << "Binding creation failed";
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, b, &spec).status,
              OVPHYSX_API_SUCCESS);

    if (spec.shape[0] > 0) {
        EXPECT_NE(try_write(m_handle, b, spec), OVPHYSX_API_SUCCESS)
            << "BODY_INV_MASS is read-only; write must be rejected";
    }

    ovphysx_destroy_tensor_binding(m_handle, b);
}

// BODY_INV_INERTIA is read-only: a write attempt must return non-SUCCESS.
TEST_F(TensorTypesExtBodyTest, BodyInvInertiaWriteRejected)
{
    ASSERT_TRUE(load_scene(m_handle, "tests/data/links_chain_sample.usda"));

    ovphysx_tensor_spec_t spec{};
    auto b = try_create(m_handle, "/World/articulation",
                        OVPHYSX_TENSOR_ARTICULATION_BODY_INV_INERTIA_F32, "BODY_INV_INERTIA");
    if (!b) GTEST_SKIP() << "Binding creation failed";
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, b, &spec).status,
              OVPHYSX_API_SUCCESS);

    if (spec.shape[0] > 0) {
        EXPECT_NE(try_write(m_handle, b, spec), OVPHYSX_API_SUCCESS)
            << "BODY_INV_INERTIA is read-only; write must be rejected";
    }

    ovphysx_destroy_tensor_binding(m_handle, b);
}
