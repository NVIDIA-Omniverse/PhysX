// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause


// Comprehensive Tensor Binding integration tests
// Tests the complete Tensor Binding API workflow including binding creation,
// write/read operations, and error handling

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <vector>
#include <cstring>

using namespace test_utils;

// Test fixture for integration tests
// Creates a per-test PhysX instance for proper test isolation
class TensorBindingIntegrationTest : public PhysXTestFixture {

    void SetUp() override {
        PhysXTestFixture::SetUp();
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));
    }

};

TEST_F(TensorBindingIntegrationTest, BindingCreationAndDestruction) {
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("/World/Body*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    desc.prim_paths = nullptr;
    desc.prim_paths_count = 0;

    ovphysx_tensor_binding_handle_t binding_handle = 0;
    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_NE(binding_handle, 0);

    ovphysx_destroy_tensor_binding(m_handle, binding_handle);
}

TEST_F(TensorBindingIntegrationTest, MultipleTensorTypes) {
    const char* paths[] = {"/World/Prim1"};
    ovphysx_string_t path_strs[] = {make_ovx_string(paths[0])};

    ovphysx_tensor_binding_desc_t desc1{};
    desc1.pattern = make_ovx_string("");
    desc1.prim_paths = path_strs;
    desc1.prim_paths_count = 1;
    desc1.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_tensor_binding_handle_t binding_handle1 = 0;
    ovphysx_result_t result1 = ovphysx_create_tensor_binding(m_handle, &desc1, &binding_handle1);
    ASSERT_EQ(result1.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(binding_handle1, 0);

    ovphysx_tensor_binding_desc_t desc2{};
    desc2.pattern = make_ovx_string("");
    desc2.prim_paths = path_strs;
    desc2.prim_paths_count = 1;
    desc2.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;

    ovphysx_tensor_binding_handle_t binding_handle2 = 0;
    ovphysx_result_t result2 = ovphysx_create_tensor_binding(m_handle, &desc2, &binding_handle2);
    ASSERT_EQ(result2.status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(binding_handle2, 0);

    ovphysx_destroy_tensor_binding(m_handle, binding_handle1);
    ovphysx_destroy_tensor_binding(m_handle, binding_handle2);
}

TEST_F(TensorBindingIntegrationTest, ReadRigidBodyPose) {
    const char* paths[] = {"/World/TestPrim1", "/World/TestPrim2"};
    ovphysx_string_t path_strs[] = {
        make_ovx_string(paths[0]),
        make_ovx_string(paths[1])
    };

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("");
    desc.prim_paths = path_strs;
    desc.prim_paths_count = 2;
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_tensor_binding_handle_t binding_handle = 0;
    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding_handle, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 2);
    EXPECT_EQ(spec.shape[1], 7); // pose is [pos(3) + quat(4)]

    std::vector<float> read_data(spec.shape[0] * spec.shape[1]);
    DLTensor read_tensor{};
    read_tensor.data = read_data.data();
    read_tensor.ndim = 2;
    read_tensor.shape = spec.shape;
    read_tensor.strides = nullptr;
    read_tensor.dtype = spec.dtype;
    read_tensor.device = {kDLCPU, 0};

    result = ovphysx_read_tensor_binding(m_handle, binding_handle, &read_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, binding_handle);
}

TEST_F(TensorBindingIntegrationTest, WriteReadRigidBodyVelocity) {
    const char* paths[] = {"/World/Cube1", "/World/Cube2"};
    ovphysx_string_t path_strs[] = {
        make_ovx_string(paths[0]),
        make_ovx_string(paths[1])
    };

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("");
    desc.prim_paths = path_strs;
    desc.prim_paths_count = 2;
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;

    ovphysx_tensor_binding_handle_t binding_handle = 0;
    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding_handle, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 2);
    EXPECT_EQ(spec.shape[1], 6); // velocity is [linear(3) + angular(3)]

    std::vector<float> write_data(spec.shape[0] * spec.shape[1]);
    // Set linear velocity to (1, 2, 3) and angular to (0.1, 0.2, 0.3) for each body
    for (int i = 0; i < spec.shape[0]; ++i) {
        write_data[i * 6 + 0] = 1.0f;  // linear x
        write_data[i * 6 + 1] = 2.0f;  // linear y
        write_data[i * 6 + 2] = 3.0f;  // linear z
        write_data[i * 6 + 3] = 0.1f;  // angular x
        write_data[i * 6 + 4] = 0.2f;  // angular y
        write_data[i * 6 + 5] = 0.3f;  // angular z
    }

    DLTensor write_tensor{};
    write_tensor.data = write_data.data();
    write_tensor.ndim = 2;
    write_tensor.shape = spec.shape;
    write_tensor.strides = nullptr;
    write_tensor.dtype = spec.dtype;
    write_tensor.device = {kDLCPU, 0};

    result = ovphysx_write_tensor_binding(m_handle, binding_handle, &write_tensor, nullptr);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> read_data(spec.shape[0] * spec.shape[1]);
    DLTensor read_tensor{};
    read_tensor.data = read_data.data();
    read_tensor.ndim = 2;
    read_tensor.shape = spec.shape;
    read_tensor.strides = nullptr;
    read_tensor.dtype = spec.dtype;
    read_tensor.device = {kDLCPU, 0};

    result = ovphysx_read_tensor_binding(m_handle, binding_handle, &read_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    for (int i = 0; i < spec.shape[0] * spec.shape[1]; ++i) {
        EXPECT_FLOAT_EQ(read_data[i], write_data[i]);
    }

    ovphysx_destroy_tensor_binding(m_handle, binding_handle);
}

TEST_F(TensorBindingIntegrationTest, ErrorHandling) {
    // Test invalid binding (neither pattern nor prim_paths specified)
    ovphysx_tensor_binding_desc_t invalid_desc{};
    invalid_desc.pattern = make_ovx_string("");
    invalid_desc.prim_paths = nullptr;
    invalid_desc.prim_paths_count = 0;
    invalid_desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_tensor_binding_handle_t binding_handle = 0;
    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &invalid_desc, &binding_handle);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT) << "Should reject empty pattern with no prim paths";

    const char* paths[] = {"/World/Test"};
    ovphysx_string_t path_strs[] = {make_ovx_string(paths[0])};

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("");
    desc.prim_paths = path_strs;
    desc.prim_paths_count = 1;
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    binding_handle = 0;
    result = ovphysx_create_tensor_binding(m_handle, &desc, &binding_handle);
    if (result.status == OVPHYSX_API_SUCCESS) {
        result = ovphysx_write_tensor_binding(m_handle, binding_handle, nullptr, nullptr);
        EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT) << "Should reject null tensor";
        ovphysx_destroy_tensor_binding(m_handle, binding_handle);
    }
}

TEST_F(TensorBindingIntegrationTest, MultipleBindings) {
    const char* paths1[] = {"/World/Prim1"};
    const char* paths2[] = {"/World/Prim2"};
    ovphysx_string_t path_strs1[] = {make_ovx_string(paths1[0])};
    ovphysx_string_t path_strs2[] = {make_ovx_string(paths2[0])};

    ovphysx_tensor_binding_desc_t desc1{};
    desc1.pattern = make_ovx_string("");
    desc1.prim_paths = path_strs1;
    desc1.prim_paths_count = 1;
    desc1.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_tensor_binding_desc_t desc2{};
    desc2.pattern = make_ovx_string("");
    desc2.prim_paths = path_strs2;
    desc2.prim_paths_count = 1;
    desc2.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;

    ovphysx_tensor_binding_handle_t handle1 = 0, handle2 = 0;
    ovphysx_result_t res1 = ovphysx_create_tensor_binding(m_handle, &desc1, &handle1);
    ovphysx_result_t res2 = ovphysx_create_tensor_binding(m_handle, &desc2, &handle2);

    EXPECT_EQ(res1.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(res2.status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, handle1);
    ovphysx_destroy_tensor_binding(m_handle, handle2);
}

TEST_F(TensorBindingIntegrationTest, PatternMatching) {
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("/World/Prim*");
    desc.prim_paths = nullptr;
    desc.prim_paths_count = 0;
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_tensor_binding_handle_t binding_handle = 0;
    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding_handle, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    // spec.shape[0] contains the number of matched prims (could be 0 if none exist)

    ovphysx_destroy_tensor_binding(m_handle, binding_handle);
}

TEST_F(TensorBindingIntegrationTest, ArticulationDOFBinding) {
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = make_ovx_string("/World/articulation*");
    desc.prim_paths = nullptr;
    desc.prim_paths_count = 0;
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_tensor_binding_handle_t binding_handle = 0;
    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding_handle, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // DOF position tensor is [N, D] where N is number of articulations, D is DOFs
    EXPECT_EQ(spec.ndim, 2);

    ovphysx_destroy_tensor_binding(m_handle, binding_handle);
}
