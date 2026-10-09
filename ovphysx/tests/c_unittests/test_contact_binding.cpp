// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// PARTIALLY DEPRECATED (tensor-binding-deprecation): the binding-isolation foil retires with the binding.

// Tests for the Contact Binding C ABI:
//   ovphysx_create_contact_binding   - null-argument error conditions
//   ovphysx_destroy_contact_binding  - invalid-handle rejection
//   ovphysx_get_contact_binding_spec - invalid-handle rejection
//   ovphysx_contact_binding_get_sensor_paths - invalid-handle rejection and
//                                              short-buffer demand report
//   ovphysx_contact_binding_get_filter_paths - invalid-handle rejection and
//                                              short-buffer demand report

//   ovphysx_get_contact_binding_capacity - invalid-handle rejection
//   ovphysx_get_contact_report       - zero-count contract before any step and
//                                      C-struct offset consistency after steps
//   ovphysx_read_raw_contact_data (with sensor + other actor IDs)
//                                     - C-ABI wiring happy path (OMPE-104131)
//
// NOTE: Happy-path contact binding lifecycle, sensor/filter counts, net-force
// shapes/values, and force-matrix shapes are already tested by the stricter
// Python suite (TestContactBinding + TestContactReport in
// tests/python_tests/cpu_tests/test_tensor_bindings_api.py). Deterministic
// per-contact identity association, overflow/truncation, and zero-contacts are
// covered at the runtime level by TestTensorContacts.cpp
// (TEST-TENSOR-CONTACT-001). Except for the opaque-handle isolation regression,
// the tests below cover only C-ABI boundary conditions that Python cannot reach.

/**
 * @implements REQ-CAPI-CONTACT-003
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-STRING-001
 * @covers AC-3
 *
 * @implements REQ-CAPI-CONTACT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-6 AC-7
 *
 * @implements REQ-CAPI-CONTACT-002
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <algorithm>
#include <string>
#include <vector>

using namespace test_utils;

// ---------------------------------------------------------------------------
// Local helpers
// ---------------------------------------------------------------------------

static bool wait_cb_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index)
{
    ovphysx_op_wait_result_t wr{};
    ovphysx_result_t r = ovphysx_wait_op(handle, op_index, 10'000'000'000ULL, &wr);
    ovphysx_destroy_wait_result(&wr);
    return r.status == OVPHYSX_API_SUCCESS;
}

static bool load_usd_cb(ovphysx_handle_t handle, const char* path, ovphysx_usd_handle_t& out)
{
    out = 1;
    return attach_usd_with_ovstage(handle, path);
}

static bool step_cb(ovphysx_handle_t handle, float elapsed)
{
    ovphysx_enqueue_result_t r = ovphysx_step(handle, elapsed);
    return r.status == OVPHYSX_API_SUCCESS && wait_cb_op(handle, r.op_index);
}

// Buffer utilities accept an entry point; they do not depend on deprecated APIs.
struct ContactDataBuffers
{
    std::vector<float> forces, points, normals, separations;
    std::vector<int32_t> counts, starts;
    ovphysx_api_status_t status;
    uint32_t required = 0;
};

static DLTensor make_contact_tensor(void* data, int64_t* shape, uint8_t code)
{
    DLTensor tensor{};
    tensor.data = data;
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {code, 32, 1};
    tensor.shape = shape;
    return tensor;
}

static ContactDataBuffers read_normal_contact_buffers(
    ovphysx_handle_t instance, ovphysx_contact_binding_handle_t binding,
    uint32_t capacity, int32_t sensors, int32_t filters, decltype(&ovphysx_read_normal_contact_data) read)
{
    ContactDataBuffers data;
    data.forces.resize(capacity);
    data.points.resize(capacity * 3);
    data.normals.resize(capacity * 3);
    data.separations.resize(capacity);
    data.counts.resize(static_cast<size_t>(sensors) * filters);
    data.starts.resize(data.counts.size());
    int64_t scalarShape[2]{capacity, 1};
    int64_t vectorShape[2]{capacity, 3};
    int64_t pairShape[2]{sensors, filters};
    DLTensor forces = make_contact_tensor(data.forces.data(), scalarShape, kDLFloat);
    DLTensor points = make_contact_tensor(data.points.data(), vectorShape, kDLFloat);
    DLTensor normals = make_contact_tensor(data.normals.data(), vectorShape, kDLFloat);
    DLTensor separations = make_contact_tensor(data.separations.data(), scalarShape, kDLFloat);
    DLTensor counts = make_contact_tensor(data.counts.data(), pairShape, kDLInt);
    DLTensor starts = make_contact_tensor(data.starts.data(), pairShape, kDLInt);
    data.status = read(instance, binding, &forces, &points, &normals, &separations,
                       &counts, &starts, &data.required).status;
    return data;
}

static ContactDataBuffers read_friction_contact_buffers(
    ovphysx_handle_t instance, ovphysx_contact_binding_handle_t binding,
    uint32_t capacity, int32_t sensors, int32_t filters, decltype(&ovphysx_read_friction_contact_data) read)
{
    ContactDataBuffers data;
    data.forces.resize(capacity * 3);
    data.points.resize(capacity * 3);
    data.counts.resize(static_cast<size_t>(sensors) * filters);
    data.starts.resize(data.counts.size());
    int64_t vectorShape[2]{capacity, 3};
    int64_t pairShape[2]{sensors, filters};
    DLTensor forces = make_contact_tensor(data.forces.data(), vectorShape, kDLFloat);
    DLTensor points = make_contact_tensor(data.points.data(), vectorShape, kDLFloat);
    DLTensor counts = make_contact_tensor(data.counts.data(), pairShape, kDLInt);
    DLTensor starts = make_contact_tensor(data.starts.data(), pairShape, kDLInt);
    data.status = read(instance, binding, &forces, &points, &counts, &starts, &data.required).status;
    return data;
}

class ContactBindingTest : public PhysXTestFixture {};

// ---------------------------------------------------------------------------
// C-ABI error conditions
// ---------------------------------------------------------------------------

// NVBug 6504951. Every ovphysx-owned object kind draws its handle from one
// process-wide, never-reused sequence. If each kind counted from 1 inside its
// own owner, an instance handle, that instance's first tensor binding and its
// first contact binding would all be the number 1 and resolve each other. For
// the valid instance below, the tensor- and contact-binding spec getters return
// OVPHYSX_API_NOT_FOUND when their resource-handle argument is absent from the
// corresponding binding map. This is the bug's reproducer plus its
// resource-to-resource variant.
TEST_F(ContactBindingTest, OpaqueHandleDomainsDoNotAlias)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_tensor_binding_desc_t tensor_desc{};
    tensor_desc.pattern = make_ovx_string("/World/Cube1");
    tensor_desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t tensor_binding = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &tensor_desc, &tensor_binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_string_t sensor = make_ovx_string("/World/Cube1");
    ovphysx_contact_binding_handle_t contact_binding = OVPHYSX_INVALID_HANDLE;
    ASSERT_EQ(ovphysx_create_contact_binding(m_handle, &sensor, 1, nullptr, 0, 256, &contact_binding).status,
              OVPHYSX_API_SUCCESS);

    EXPECT_NE(tensor_binding, contact_binding);
    EXPECT_NE(tensor_binding, m_handle);
    EXPECT_NE(contact_binding, m_handle);

    ovphysx_tensor_spec_t tensor_spec{};
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(m_handle, tensor_binding, &tensor_spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(m_handle, contact_binding, &tensor_spec).status, OVPHYSX_API_NOT_FOUND);
    // The exact lookup from the bug: passing the valid instance handle where a
    // tensor-binding handle is expected must not resolve the first tensor
    // binding and populate tensor_spec from it.
    EXPECT_EQ(ovphysx_get_tensor_binding_spec(m_handle, m_handle, &tensor_spec).status, OVPHYSX_API_NOT_FOUND);

    int32_t sensor_count = 0;
    int32_t filter_count = 0;
    EXPECT_EQ(ovphysx_get_contact_binding_spec(m_handle, contact_binding, &sensor_count, &filter_count).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_get_contact_binding_spec(m_handle, tensor_binding, &sensor_count, &filter_count).status,
              OVPHYSX_API_NOT_FOUND);

    EXPECT_EQ(ovphysx_destroy_contact_binding(m_handle, contact_binding).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, tensor_binding).status, OVPHYSX_API_SUCCESS);
}

// get_spec verifies the sensor/filter counts returned by the API:
//   sensor_count - number of rigid-body prims matched by the sensor pattern
//                  that also have the contact-report API enabled.
//                  boxes_falling_on_groundplane.usda has Cube1..3 with contact
//                  report enabled, so /World/Cube* must yield sensor_count >= 3.
//   filter_count - number of filter patterns registered at binding creation
//                  (not matched prims). One filter pattern is passed, so
//                  filter_count must equal exactly 1.
TEST_F(ContactBindingTest, GetContactBindingSpecMinimumCounts)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensor  = make_ovx_string("/World/Cube*");
    ovphysx_string_t filter  = make_ovx_string("/World/Cube*");
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, &sensor, 1, &filter, 1, 256, &cb);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create contact binding";

    int32_t sensor_count = -1, filter_count = -1;
    r = ovphysx_get_contact_binding_spec(m_handle, cb, &sensor_count, &filter_count);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // sensor_count = matched rigid bodies with contact-report API enabled.
    EXPECT_GE(sensor_count, 3)
        << "expected at least 3 sensors matching /World/Cube* (Cube1..3 have "
           "contact report API); got " << sensor_count;

    // filter_count = number of filter patterns registered (not matched prims).
    // One filter pattern string was registered, so the API must return exactly 1.
    EXPECT_EQ(filter_count, 1)
        << "expected filter_count == 1 (one filter pattern was registered); "
           "got " << filter_count;

    ovphysx_destroy_contact_binding(m_handle, cb);
}

// destroy with invalid handle must reject, not crash.
TEST_F(ContactBindingTest, DestroyInvalidHandle)
{
    ovphysx_result_t r = ovphysx_destroy_contact_binding(m_handle, OVPHYSX_INVALID_HANDLE);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// get_spec with invalid contact binding handle must reject.
TEST_F(ContactBindingTest, GetSpecInvalidHandle)
{
    int32_t sc = -1, fc = -1;
    ovphysx_result_t r = ovphysx_get_contact_binding_spec(
        m_handle, OVPHYSX_INVALID_HANDLE, &sc, &fc);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// get_capacity with invalid contact binding handle must reject.
TEST_F(ContactBindingTest, GetCapacityInvalidHandle)
{
    uint32_t capacity = 0;
    ovphysx_result_t r = ovphysx_get_contact_binding_capacity(
        m_handle, OVPHYSX_INVALID_HANDLE, &capacity);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// get_sensor_paths with invalid contact binding handle must reject.
TEST_F(ContactBindingTest, GetSensorPathsInvalidHandle)
{
    ovphysx_string_t paths[1]{};
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_contact_binding_get_sensor_paths(
        m_handle, OVPHYSX_INVALID_HANDLE, paths, 1, &count);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// get_filter_paths with invalid contact binding handle must reject.
TEST_F(ContactBindingTest, GetFilterPathsInvalidHandle)
{
    ovphysx_string_t paths[1]{};
    uint32_t count = 0;
    ovphysx_result_t r = ovphysx_contact_binding_get_filter_paths(
        m_handle, OVPHYSX_INVALID_HANDLE, paths, 1, &count);
    EXPECT_NE(r.status, OVPHYSX_API_SUCCESS);
}

// get_capacity with null output pointer must return INVALID_ARGUMENT.
TEST_F(ContactBindingTest, GetCapacityNullOut)
{
    ovphysx_result_t r = ovphysx_get_contact_binding_capacity(
        m_handle, OVPHYSX_INVALID_HANDLE, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// create with null out_handle must return INVALID_ARGUMENT.
TEST_F(ContactBindingTest, CreateWithNullOutHandle)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensor = make_ovx_string("/World/Cube1");
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, 256, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// create with null sensors array must return INVALID_ARGUMENT.
TEST_F(ContactBindingTest, CreateWithNullSensors)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, nullptr, 1, nullptr, 0, 256, &cb);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// NVBugs 6433621: embedded NUL bytes in contact patterns must be rejected.
TEST_F(ContactBindingTest, RejectsEmbeddedNulSensorPattern)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    std::string storage;
    ovphysx_string_t sensor = make_ovx_string_bytes(
        std::string("/World/Cube1") + '\0' + "GARBAGE", storage);
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, 256, &cb);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(ContactBindingTest, RejectsEmbeddedNulFilterPattern)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensor = make_ovx_string("/World/Cube1");
    std::string storage;
    ovphysx_string_t filter = make_ovx_string_bytes(
        std::string("/World/Cube2") + '\0' + "GARBAGE", storage);
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, &sensor, 1, &filter, 1, 256, &cb);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// get_spec returns exactly the sensor and filter counts registered at creation.
// Two explicit sensor paths (Cube1, Cube2) and two filter patterns per sensor
// (Cube2, Cube3) are passed.  The flat filter array has sensor_count *
// filters_per_sensor = 2 * 2 = 4 entries (same 2 patterns for each sensor).
// The spec must reflect those exact counts. A permissive >= 0 check would
// pass even if registration silently dropped entries, so EXPECT_EQ is used to
// catch any mis-registration at the C-ABI boundary.
TEST_F(ContactBindingTest, GetContactBindingSpecExactCounts)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensors[2] = {
        make_ovx_string("/World/Cube1"),
        make_ovx_string("/World/Cube2"),
    };
    // filters_per_sensor = 2, sensor_count = 2  =>  4 entries in flat array
    // (layout: [sensor0_filter0, sensor0_filter1, sensor1_filter0, sensor1_filter1])
    ovphysx_string_t filters[4] = {
        make_ovx_string("/World/Cube2"),
        make_ovx_string("/World/Cube3"),
        make_ovx_string("/World/Cube2"),
        make_ovx_string("/World/Cube3"),
    };
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, sensors, 2, filters, 2, 256, &cb);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create contact binding";

    int32_t sensor_count = -1, filter_count = -1;
    r = ovphysx_get_contact_binding_spec(m_handle, cb, &sensor_count, &filter_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(sensor_count, 2)
        << "expected exactly 2 sensors (Cube1, Cube2); got " << sensor_count;
    EXPECT_EQ(filter_count, 2)
        << "expected exactly 2 filters (Cube2, Cube3); got " << filter_count;

    ovphysx_string_t sensor_paths[2]{};
    uint32_t path_count = 0;
    r = ovphysx_contact_binding_get_sensor_paths(m_handle, cb, sensor_paths, 2, &path_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(path_count, 2u);
    for (uint32_t index = 0; index < path_count; ++index)
    {
        ASSERT_NE(sensor_paths[index].ptr, nullptr);
        EXPECT_EQ(sensor_paths[index].ptr[sensor_paths[index].length], '\0');
    }
    EXPECT_EQ(std::string(sensor_paths[0].ptr, sensor_paths[0].length), "/World/Cube1");
    EXPECT_EQ(std::string(sensor_paths[1].ptr, sensor_paths[1].length), "/World/Cube2");

    ovphysx_string_t filter_paths[4]{};
    r = ovphysx_contact_binding_get_filter_paths(m_handle, cb, filter_paths, 4, &path_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(path_count, 4u);
    for (uint32_t index = 0; index < path_count; ++index)
    {
        ASSERT_NE(filter_paths[index].ptr, nullptr);
        EXPECT_EQ(filter_paths[index].ptr[filter_paths[index].length], '\0');
    }
    EXPECT_EQ(std::string(filter_paths[0].ptr, filter_paths[0].length), "/World/Cube2");
    EXPECT_EQ(std::string(filter_paths[1].ptr, filter_paths[1].length), "/World/Cube3");
    EXPECT_EQ(std::string(filter_paths[2].ptr, filter_paths[2].length), "/World/Cube2");
    EXPECT_EQ(std::string(filter_paths[3].ptr, filter_paths[3].length), "/World/Cube3");

    ovphysx_destroy_contact_binding(m_handle, cb);
}

TEST_F(ContactBindingTest, PathGettersReportDemandAndBufferTooSmall)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensors[2] = {
        make_ovx_string("/World/Cube1"),
        make_ovx_string("/World/Cube2"),
    };
    ovphysx_string_t filters[4] = {
        make_ovx_string("/World/Cube2"),
        make_ovx_string("/World/Cube3"),
        make_ovx_string("/World/Cube2"),
        make_ovx_string("/World/Cube3"),
    };
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, sensors, 2, filters, 2, 256, &cb);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create contact binding";

    ovphysx_string_t sensor_prefix[1]{};
    uint32_t path_count = 0;
    r = ovphysx_contact_binding_get_sensor_paths(m_handle, cb, sensor_prefix, 1, &path_count);
    EXPECT_EQ(r.status, OVPHYSX_API_BUFFER_TOO_SMALL);
    ASSERT_EQ(path_count, 2u);
    ASSERT_NE(sensor_prefix[0].ptr, nullptr);
    EXPECT_EQ(std::string(sensor_prefix[0].ptr, sensor_prefix[0].length), "/World/Cube1");

    ovphysx_string_t filter_prefix[1]{};
    path_count = 0;
    r = ovphysx_contact_binding_get_filter_paths(m_handle, cb, filter_prefix, 1, &path_count);
    EXPECT_EQ(r.status, OVPHYSX_API_BUFFER_TOO_SMALL);
    ASSERT_EQ(path_count, 4u);
    ASSERT_NE(filter_prefix[0].ptr, nullptr);
    EXPECT_EQ(std::string(filter_prefix[0].ptr, filter_prefix[0].length), "/World/Cube2");

    ovphysx_destroy_contact_binding(m_handle, cb);
}

TEST_F(ContactBindingTest, DetailedReadsRejectUnfilteredBinding)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensor = make_ovx_string("/World/Cube1");
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, 256, &cb);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create unfiltered contact binding";

    int32_t sensor_count = -1, filter_count = -1;
    r = ovphysx_get_contact_binding_spec(m_handle, cb, &sensor_count, &filter_count);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(sensor_count, 0);
    ASSERT_EQ(filter_count, 0);

    DLTensor* contact_forces = make_float32_tensor(std::vector<float>(256), {256, 1});
    DLTensor* positions = make_float32_tensor(std::vector<float>(256 * 3), {256, 3});
    DLTensor* normals = make_float32_tensor(std::vector<float>(256 * 3), {256, 3});
    DLTensor* separations = make_float32_tensor(std::vector<float>(256), {256, 1});
    DLTensor* counts = make_int32_tensor({}, {sensor_count, filter_count});
    DLTensor* starts = make_int32_tensor({}, {sensor_count, filter_count});
    DLTensor* friction_forces = make_float32_tensor(std::vector<float>(256 * 3), {256, 3});
    DLTensor* friction_points = make_float32_tensor(std::vector<float>(256 * 3), {256, 3});
    uint32_t required_contact_count = 0;
    uint32_t required_friction_count = 0;

    r = ovphysx_read_normal_contact_data(
        m_handle, cb, contact_forces, positions, normals, separations, counts, starts, &required_contact_count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);

    r = ovphysx_read_friction_contact_data(
        m_handle, cb, friction_forces, friction_points, counts, starts, &required_friction_count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);

    free_tensor(contact_forces);
    free_tensor(positions);
    free_tensor(normals);
    free_tensor(separations);
    free_tensor(counts);
    free_tensor(starts);
    free_tensor(friction_forces);
    free_tensor(friction_points);
    ovphysx_destroy_contact_binding(m_handle, cb);
}

// Overflow reports the complete demand while preserving a usable prefix.
TEST_F(ContactBindingTest, FilteredReadsOverflowReturnsRequiredCountAndValidPrefix)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensor = make_ovx_string("/World/Cube*");
    ovphysx_string_t filter = make_ovx_string("/World/BigBase");
    constexpr uint32_t kSmallCapacity = 1;
    constexpr uint32_t kReferenceCapacity = 64;
    ovphysx_contact_binding_handle_t small_cb = 0;
    ovphysx_contact_binding_handle_t reference_cb = 0;
    ASSERT_EQ(ovphysx_create_contact_binding(
        m_handle, &sensor, 1, &filter, 1, kSmallCapacity, &small_cb).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_contact_binding(
        m_handle, &sensor, 1, &filter, 1, kReferenceCapacity, &reference_cb).status,
        OVPHYSX_API_SUCCESS);

    int32_t sensor_count = -1;
    int32_t filter_count = -1;
    ASSERT_EQ(ovphysx_get_contact_binding_spec(
        m_handle, small_cb, &sensor_count, &filter_count).status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(sensor_count, 0);
    ASSERT_GT(filter_count, 0);

    for (int i = 0; i < 60; ++i)
        ASSERT_TRUE(step_cb(m_handle, 1.0f / 60.0f));

    struct LayoutSummary
    {
        ovphysx_api_status_t status;
        uint32_t required;
        uint32_t written;
        uint32_t maxEnd;
    };

    auto summarize = [&](const std::vector<int32_t>& counts, const std::vector<int32_t>& starts,
                         ovphysx_api_status_t status, uint32_t required)
    {
        uint32_t written = 0;
        uint32_t max_end = 0;
        const size_t pairs = static_cast<size_t>(sensor_count) * static_cast<size_t>(filter_count);
        for (size_t i = 0; i < pairs; ++i)
        {
            if (counts[i] < 0 || starts[i] < 0)
            {
                ADD_FAILURE() << "negative layout at pair " << i
                              << " count=" << counts[i] << " start=" << starts[i];
                written = UINT32_MAX;
                max_end = UINT32_MAX;
                break;
            }
            const uint32_t count = static_cast<uint32_t>(counts[i]);
            const uint32_t start = static_cast<uint32_t>(starts[i]);
            written += count;
            max_end = std::max(max_end, start + count);
        }
        return LayoutSummary{status, required, written, max_end};
    };

    auto read_contacts = [&](ovphysx_contact_binding_handle_t binding, uint32_t capacity)
    {
        const ContactDataBuffers data = read_normal_contact_buffers(
            m_handle, binding, capacity, sensor_count, filter_count, ovphysx_read_normal_contact_data);
        return summarize(data.counts, data.starts, data.status, data.required);
    };
    auto read_friction = [&](ovphysx_contact_binding_handle_t binding, uint32_t capacity)
    {
        const ContactDataBuffers data = read_friction_contact_buffers(
            m_handle, binding, capacity, sensor_count, filter_count, ovphysx_read_friction_contact_data);
        return summarize(data.counts, data.starts, data.status, data.required);
    };

    // Read both bindings against the same settled step so contact counts cannot
    // drift between the reference and undersized reads.
    const LayoutSummary contact_reference = read_contacts(reference_cb, kReferenceCapacity);
    ASSERT_EQ(contact_reference.status, OVPHYSX_API_SUCCESS) << ovphysx_get_last_error().ptr;
    ASSERT_GT(contact_reference.required, kSmallCapacity);
    EXPECT_EQ(contact_reference.written, contact_reference.required);

    const LayoutSummary contact_small = read_contacts(small_cb, kSmallCapacity);
    EXPECT_EQ(contact_small.status, OVPHYSX_API_BUFFER_TOO_SMALL);
    EXPECT_EQ(contact_small.required, contact_reference.required);
    EXPECT_EQ(contact_small.written, kSmallCapacity);
    EXPECT_LE(contact_small.maxEnd, kSmallCapacity);

    const LayoutSummary friction_reference = read_friction(reference_cb, kReferenceCapacity);
    ASSERT_EQ(friction_reference.status, OVPHYSX_API_SUCCESS) << ovphysx_get_last_error().ptr;
    ASSERT_GT(friction_reference.required, kSmallCapacity);
    EXPECT_EQ(friction_reference.written, friction_reference.required);

    const LayoutSummary friction_small = read_friction(small_cb, kSmallCapacity);
    EXPECT_EQ(friction_small.status, OVPHYSX_API_BUFFER_TOO_SMALL);
    EXPECT_EQ(friction_small.required, friction_reference.required);
    EXPECT_EQ(friction_small.written, kSmallCapacity);
    EXPECT_LE(friction_small.maxEnd, kSmallCapacity);

    ovphysx_destroy_contact_binding(m_handle, small_cb);
    ovphysx_destroy_contact_binding(m_handle, reference_cb);
}

// ---------------------------------------------------------------------------
// get_contact_report - contract tests not reachable from Python
// ---------------------------------------------------------------------------


// Before any simulation step the report must be empty (both counts == 0).
// Python test_no_contacts_before_collision only asserts >= 0. The C ABI
// must guarantee the tighter contract of exactly 0 at this stage.
TEST_F(ContactBindingTest, GetContactReportBeforeStep)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    const ovphysx_contact_event_header_t* headers = nullptr;
    uint32_t num_headers = 0;
    const ovphysx_contact_point_t* points = nullptr;
    uint32_t num_points = 0;

    ovphysx_result_t r = ovphysx_get_contact_report(
        m_handle, &headers, &num_headers, &points, &num_points,
        nullptr, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(num_headers, 0u) << "No contacts expected before any simulation step";
    EXPECT_EQ(num_points,  0u) << "No contact points expected before any simulation step";
}

// After stepping the simulation, two things are validated:
//  1. Contacts were detected (non-empty report). Without this the consistency
//     loop below is vacuously satisfied and catches nothing.
//  2. C-struct internal consistency: every header's
//     (contactDataOffset + numContactData) must not exceed num_points.
//     This is a C-ABI-level invariant Python cannot check because it works
//     with the Python wrapper's dict, not the raw structs.
//
// NOTE: A contact binding covering the sensor prims must be created before
// stepping. Without a binding the engine has no sensor list and produces
// zero contact headers regardless of how many steps are taken.
TEST_F(ContactBindingTest, GetContactReportStructConsistencyAfterSteps)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    // Register a contact binding so the engine tracks contacts for Cube1..3.
    ovphysx_string_t sensor = make_ovx_string("/World/Cube*");
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t cr = ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, 256, &cb);
    ASSERT_EQ(cr.status, OVPHYSX_API_SUCCESS) << "Failed to create contact binding";

    // 60 steps at 1/60 s is one second of simulation. This matches the Python
    // test_contacts_after_falling step count and is enough for the boxes to
    // reach the ground plane under gravity.
    for (int i = 0; i < 60; ++i)
        ASSERT_TRUE(step_cb(m_handle, 1.0f / 60.0f));

    const ovphysx_contact_event_header_t* headers = nullptr;
    uint32_t num_headers = 0;
    const ovphysx_contact_point_t* points = nullptr;
    uint32_t num_points = 0;

    ovphysx_result_t r = ovphysx_get_contact_report(
        m_handle, &headers, &num_headers, &points, &num_points,
        nullptr, nullptr);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    // Contacts must have been detected. If zero headers are returned the
    // consistency loop below never runs and the test is vacuous.
    ASSERT_GT(num_headers, 0u)
        << "Expected at least one contact pair after boxes fall onto the "
           "ground plane (60 simulation steps at 1/60 s each)";
    ASSERT_GT(num_points, 0u)
        << "Expected at least one contact point after boxes fall onto the "
           "ground plane (60 simulation steps at 1/60 s each)";

    // The headers pointer must be valid since num_headers > 0.
    ASSERT_NE(headers, nullptr);

    // Validate every header's contact-data range falls within [0, num_points).
    for (uint32_t i = 0; i < num_headers; ++i)
    {
        uint32_t end = headers[i].contactDataOffset + headers[i].numContactData;
        EXPECT_LE(end, num_points)
            << "header[" << i << "].contactDataOffset("
            << headers[i].contactDataOffset << ") + numContactData("
            << headers[i].numContactData << ") = " << end
            << " exceeds num_points(" << num_points << ")";
    }

    ovphysx_destroy_contact_binding(m_handle, cb);
}

// ---------------------------------------------------------------------------
// ovphysx_read_raw_contact_data + ovphysx_contact_binding_get_other_actor_paths_from_ids
// (OMPE-94459 #21 wire-up)
// ---------------------------------------------------------------------------

// Raw contact data is the filter-less variant of read_normal_contact_data: per-sensor
// counts/start-indices are 1D [S] (no filter dim) and each contact carries an
// opaque actor id resolvable to a USD prim path. This test exercises the
// happy path on stacked boxes after enough simulation steps for the stack to
// settle.
TEST_F(ContactBindingTest, RawContactDataAndOtherActorPaths)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    // Filter-less binding: filters_per_sensor = 0 is valid for raw reads.
    ovphysx_string_t sensor = make_ovx_string("/World/Cube*");
    constexpr uint32_t kMaxContacts = 64;
    ovphysx_contact_binding_handle_t cb = 0;
    ovphysx_result_t r = ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, kMaxContacts, &cb);
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    int32_t sensor_count = -1, filter_count = -1;
    ASSERT_EQ(ovphysx_get_contact_binding_spec(m_handle, cb, &sensor_count, &filter_count).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_GT(sensor_count, 0);

    // Settle the stack so contacts exist to read.
    for (int i = 0; i < 60; ++i)
        ASSERT_TRUE(step_cb(m_handle, 1.0f / 60.0f));

    std::vector<float> force_buf(kMaxContacts * 1, 0.0f);
    std::vector<float> point_buf(kMaxContacts * 3, 0.0f);
    std::vector<float> normal_buf(kMaxContacts * 3, 0.0f);
    std::vector<float> separation_buf(kMaxContacts * 1, 0.0f);
    // (S, 2): column 0 count, column 1 start index.
    std::vector<int32_t> layout_buf(sensor_count * 2, 0);
    // (C, 2): column 0 sensor actor, column 1 other actor.
    std::vector<int64_t> ids_buf(kMaxContacts * 2, 0);

    auto make_tensor = [](void* data, int ndim, int64_t* shape, uint8_t code, uint8_t bits) {
        DLTensor t{};
        t.data = data;
        t.device = {kDLCPU, 0};
        t.ndim = ndim;
        t.dtype = {code, bits, 1};
        t.shape = shape;
        return t;
    };

    int64_t cshape[2] = {kMaxContacts, 1};
    int64_t pshape[2] = {kMaxContacts, 3};
    int64_t sshape[2] = {sensor_count, 2};
    int64_t ishape[2] = {kMaxContacts, 2};
    DLTensor force_t      = make_tensor(force_buf.data(),       2, cshape, kDLFloat, 32);
    DLTensor point_t      = make_tensor(point_buf.data(),       2, pshape, kDLFloat, 32);
    DLTensor normal_t     = make_tensor(normal_buf.data(),      2, pshape, kDLFloat, 32);
    DLTensor separation_t = make_tensor(separation_buf.data(),  2, cshape, kDLFloat, 32);
    DLTensor layout_t     = make_tensor(layout_buf.data(),      2, sshape, kDLInt,   32);
    DLTensor ids_t        = make_tensor(ids_buf.data(),         2, ishape, kDLInt,   64);

    uint32_t required_contact_count = 0;
    r = ovphysx_read_raw_contact_data(
        m_handle, cb, &force_t, &point_t, &normal_t, &separation_t, &layout_t, &ids_t,
        &required_contact_count);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS) << ovphysx_get_last_error().ptr;

    // At least one sensor must have at least one contact after settling.
    int32_t total_contacts = 0;
    for (int32_t i = 0; i < sensor_count; ++i) total_contacts += layout_buf[i * 2 + 0];
    EXPECT_GT(total_contacts, 0) << "expected at least one contact after 60 steps";
    EXPECT_EQ(required_contact_count, static_cast<uint32_t>(total_contacts));

    // Every written sensor/other id within [start, start+count) is non-zero.
    for (int32_t i = 0; i < sensor_count; ++i)
    {
        const int32_t count = layout_buf[static_cast<size_t>(i) * 2 + 0];
        const int32_t start = layout_buf[static_cast<size_t>(i) * 2 + 1];
        for (int32_t k = start; k < start + count && k < static_cast<int32_t>(kMaxContacts); ++k)
        {
            EXPECT_NE(ids_buf[static_cast<size_t>(k) * 2 + 0], 0);
            EXPECT_NE(ids_buf[static_cast<size_t>(k) * 2 + 1], 0);
        }
    }

    // The resolver takes either id buffer, because both use one namespace (REQ AC-4).
    // Each is resolved and every id that was written must yield a non-empty path.
    // A zero id (the untouched tail of the buffer) resolves to empty.
    auto resolve_and_check = [&](DLTensor& ids_tensor, const int64_t* ids_buf, const char* which)
    {
        std::vector<ovphysx_string_t> paths(kMaxContacts);
        uint32_t written = 0;
        ovphysx_result_t rr = ovphysx_contact_binding_get_other_actor_paths_from_ids(
            m_handle, cb, &ids_tensor, paths.data(), kMaxContacts, &written);
        EXPECT_EQ(rr.status, OVPHYSX_API_SUCCESS) << which << ": " << ovphysx_get_last_error().ptr;
        EXPECT_EQ(written, kMaxContacts) << which << ": one path per id";
        for (uint32_t index = 0; index < written; ++index)
        {
            ASSERT_NE(paths[index].ptr, nullptr) << which << " at " << index;
            EXPECT_EQ(paths[index].ptr[paths[index].length], '\0') << which << " at " << index;
            if (ids_buf[index] != 0)
            {
                EXPECT_GT(paths[index].length, 0u)
                    << which << ": non-zero id at " << index << " must resolve to a path";
            }
        }
    };
    // The resolver takes a flat id list, so each column is pulled out of the (C, 2)
    // tensor first, the same slice a caller would take (numpy `ids[:, 0]` / `ids[:, 1]`).
    std::vector<int64_t> sensor_col(kMaxContacts, 0), other_col(kMaxContacts, 0);
    for (uint32_t k = 0; k < kMaxContacts; ++k)
    {
        sensor_col[k] = ids_buf[k * 2 + 0];
        other_col[k] = ids_buf[k * 2 + 1];
    }
    int64_t colshape[1] = {kMaxContacts};
    DLTensor sensor_col_t = make_tensor(sensor_col.data(), 1, colshape, kDLInt, 64);
    DLTensor other_col_t = make_tensor(other_col.data(), 1, colshape, kDLInt, 64);

    // Each call replaces the binding's path cache, so the previous call's pointers
    // are dead by the time the next one returns. Each result is checked before the
    // next call.
    resolve_and_check(other_col_t, other_col.data(), "other_actor_ids");
    resolve_and_check(sensor_col_t, sensor_col.data(), "sensor_actor_ids");

    {
        std::vector<ovphysx_string_t> short_paths(1);
        uint32_t demand = 0;
        ovphysx_result_t truncated = ovphysx_contact_binding_get_other_actor_paths_from_ids(
            m_handle, cb, &other_col_t, short_paths.data(), 1, &demand);
        EXPECT_EQ(truncated.status, OVPHYSX_API_BUFFER_TOO_SMALL)
            << ovphysx_get_last_error().ptr;
        EXPECT_EQ(demand, kMaxContacts);
        ASSERT_NE(short_paths[0].ptr, nullptr);
        if (other_col[0] != 0)
        {
            EXPECT_GT(short_paths[0].length, 0u);
        }
    }

    ovphysx_destroy_contact_binding(m_handle, cb);
}

// Overflow reports the complete demand while preserving a usable prefix.
TEST_F(ContactBindingTest, RawContactDataOverflowReturnsRequiredCountAndValidPrefix)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensor = make_ovx_string("/World/Cube*");
    constexpr uint32_t kSmallCapacity = 1;
    constexpr uint32_t kReferenceCapacity = 64;
    ovphysx_contact_binding_handle_t small_cb = 0;
    ovphysx_contact_binding_handle_t reference_cb = 0;
    ASSERT_EQ(ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, kSmallCapacity, &small_cb).status,
        OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, kReferenceCapacity, &reference_cb).status,
        OVPHYSX_API_SUCCESS);

    int32_t sensor_count = -1;
    int32_t filter_count = -1;
    ASSERT_EQ(ovphysx_get_contact_binding_spec(
        m_handle, small_cb, &sensor_count, &filter_count).status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(sensor_count, 0);

    for (int i = 0; i < 60; ++i)
        ASSERT_TRUE(step_cb(m_handle, 1.0f / 60.0f));

    struct ReadSummary
    {
        ovphysx_api_status_t status;
        uint32_t required;
        uint32_t written;
        uint32_t maxEnd;
        int64_t firstSensorId;
        int64_t firstOtherId;
    };

    auto read = [&](ovphysx_contact_binding_handle_t binding, uint32_t capacity)
    {
        std::vector<float> forces(capacity, 0.0f);
        std::vector<float> points(capacity * 3, 0.0f);
        std::vector<float> normals(capacity * 3, 0.0f);
        std::vector<float> separations(capacity, 0.0f);
        std::vector<int32_t> layout(static_cast<size_t>(sensor_count) * 2, 0);
        std::vector<int64_t> ids(static_cast<size_t>(capacity) * 2, 0);

        int64_t scalar_shape[2] = {capacity, 1};
        int64_t vector_shape[2] = {capacity, 3};
        int64_t layout_shape[2] = {sensor_count, 2};
        int64_t ids_shape[2] = {capacity, 2};
        auto make_tensor = [](void* data, int64_t* shape, uint8_t code, uint8_t bits)
        {
            DLTensor tensor{};
            tensor.data = data;
            tensor.device = {kDLCPU, 0};
            tensor.ndim = 2;
            tensor.dtype = {code, bits, 1};
            tensor.shape = shape;
            return tensor;
        };
        DLTensor force_tensor = make_tensor(forces.data(), scalar_shape, kDLFloat, 32);
        DLTensor point_tensor = make_tensor(points.data(), vector_shape, kDLFloat, 32);
        DLTensor normal_tensor = make_tensor(normals.data(), vector_shape, kDLFloat, 32);
        DLTensor separation_tensor = make_tensor(separations.data(), scalar_shape, kDLFloat, 32);
        DLTensor layout_tensor = make_tensor(layout.data(), layout_shape, kDLInt, 32);
        DLTensor ids_tensor = make_tensor(ids.data(), ids_shape, kDLInt, 64);

        uint32_t required = 0;
        const ovphysx_result_t result = ovphysx_read_raw_contact_data(
            m_handle, binding, &force_tensor, &point_tensor, &normal_tensor,
            &separation_tensor, &layout_tensor, &ids_tensor, &required);

        uint32_t written = 0;
        uint32_t max_end = 0;
        for (int32_t i = 0; i < sensor_count; ++i)
        {
            const uint32_t count = static_cast<uint32_t>(layout[static_cast<size_t>(i) * 2]);
            const uint32_t start = static_cast<uint32_t>(layout[static_cast<size_t>(i) * 2 + 1]);
            written += count;
            max_end = std::max(max_end, start + count);
        }
        return ReadSummary{
            result.status,
            required,
            written,
            max_end,
            ids[0],
            ids[1],
        };
    };

    // Read both bindings against the same settled step -- getRawContactData is a
    // non-destructive read of the last step's contact data, so re-stepping between
    // the two reads would let contact counts drift (they are not guaranteed to be
    // bit-stable step over step) and make small.required != reference.required a flake.
    const ReadSummary reference = read(reference_cb, kReferenceCapacity);
    ASSERT_EQ(reference.status, OVPHYSX_API_SUCCESS) << ovphysx_get_last_error().ptr;
    ASSERT_GT(reference.required, kSmallCapacity);
    EXPECT_EQ(reference.written, reference.required);

    const ReadSummary small = read(small_cb, kSmallCapacity);
    EXPECT_EQ(small.status, OVPHYSX_API_BUFFER_TOO_SMALL);
    EXPECT_EQ(small.required, reference.required);
    EXPECT_EQ(small.written, kSmallCapacity);
    EXPECT_LE(small.maxEnd, kSmallCapacity);
    EXPECT_NE(small.firstSensorId, 0);
    EXPECT_NE(small.firstOtherId, 0);

    ovphysx_destroy_contact_binding(m_handle, small_cb);
    ovphysx_destroy_contact_binding(m_handle, reference_cb);
}

// Shape-mismatch rejection: count tensor with the wrong dim must be rejected.
TEST_F(ContactBindingTest, RawContactDataRejectsWrongShape)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle,
        "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_string_t sensor = make_ovx_string("/World/Cube*");
    constexpr uint32_t kMaxContacts = 16;
    ovphysx_contact_binding_handle_t cb = 0;
    ASSERT_EQ(ovphysx_create_contact_binding(
        m_handle, &sensor, 1, nullptr, 0, kMaxContacts, &cb).status, OVPHYSX_API_SUCCESS);

    int32_t sensor_count = -1, filter_count = -1;
    ASSERT_EQ(ovphysx_get_contact_binding_spec(m_handle, cb, &sensor_count, &filter_count).status,
              OVPHYSX_API_SUCCESS);

    std::vector<float> force_buf(kMaxContacts, 0.0f);
    std::vector<float> point_buf(kMaxContacts * 3, 0.0f);
    std::vector<float> normal_buf(kMaxContacts * 3, 0.0f);
    std::vector<float> separation_buf(kMaxContacts, 0.0f);
    // Intentionally [S, 4]. read_raw_contact_data requires the [S, 2] sensor layout.
    std::vector<int32_t> layout_buf(sensor_count * 4, 0);
    std::vector<int64_t> ids_buf2(kMaxContacts * 2, 0);

    int64_t cshape[2] = {kMaxContacts, 1};
    int64_t pshape[2] = {kMaxContacts, 3};
    int64_t layout_bad_shape[2] = {sensor_count, 4};
    int64_t ishape[2] = {kMaxContacts, 2};
    auto make_tensor = [](void* data, int ndim, int64_t* shape, uint8_t code, uint8_t bits) {
        DLTensor t{};
        t.data = data; t.device = {kDLCPU, 0}; t.ndim = ndim;
        t.dtype = {code, bits, 1}; t.shape = shape;
        return t;
    };
    DLTensor force_t        = make_tensor(force_buf.data(),        2, cshape, kDLFloat, 32);
    DLTensor point_t        = make_tensor(point_buf.data(),        2, pshape, kDLFloat, 32);
    DLTensor normal_t       = make_tensor(normal_buf.data(),       2, pshape, kDLFloat, 32);
    DLTensor separation_t   = make_tensor(separation_buf.data(),   2, cshape, kDLFloat, 32);
    DLTensor layout_t       = make_tensor(layout_buf.data(),       2, layout_bad_shape, kDLInt, 32);
    DLTensor ids_t2         = make_tensor(ids_buf2.data(),         2, ishape, kDLInt, 64);

    uint32_t required_contact_count = 0;
    ovphysx_result_t r = ovphysx_read_raw_contact_data(
        m_handle, cb, &force_t, &point_t, &normal_t, &separation_t, &layout_t, &ids_t2,
        &required_contact_count);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "expected shape-mismatch rejection (sensor layout was [S, 4], raw read wants [S, 2])";

    ovphysx_destroy_contact_binding(m_handle, cb);
}

// Component reads retain the C boundary contract independently of compatibility aliases.
TEST_F(ContactBindingTest, ForceComponents)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_cb(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));
    ovphysx_string_t sensor = make_ovx_string("/World/Cube1");
    ovphysx_string_t filter = make_ovx_string("/World/BigBase");
    ovphysx_contact_binding_handle_t cb = 0;
    ASSERT_EQ(ovphysx_create_contact_binding(m_handle, &sensor, 1, &filter, 1, 0, &cb).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(step_cb(m_handle, 1.0f / 60.0f));

    float normal[3]{}, friction[3]{}, matrix[3]{};
    int64_t netShape[2]{1, 3};
    int64_t matrixShape[3]{1, 1, 3};
    DLTensor dst{};
    dst.data = normal;
    dst.device = {kDLCPU, 0};
    dst.dtype = {kDLFloat, 32, 1};
    dst.ndim = 2;
    dst.shape = netShape;
    ASSERT_EQ(ovphysx_read_contact_net_normal_forces(m_handle, cb, &dst).status, OVPHYSX_API_SUCCESS);
    EXPECT_GT(normal[2], 0.0f);
    dst.data = friction;
    ASSERT_EQ(ovphysx_read_contact_net_friction_forces(m_handle, cb, &dst).status, OVPHYSX_API_SUCCESS);
    // Friction on this horizontal support must not include its normal force.
    EXPECT_NEAR(friction[2], 0.0f, 0.1f);

    EXPECT_EQ(ovphysx_read_contact_net_friction_forces(m_handle, cb, nullptr).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    dst.dtype.code = kDLInt;
    EXPECT_EQ(ovphysx_read_contact_net_friction_forces(m_handle, cb, &dst).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    dst.dtype.code = kDLFloat;
    netShape[1] = 2;
    EXPECT_EQ(ovphysx_read_contact_net_friction_forces(m_handle, cb, &dst).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    netShape[1] = 3;
    dst.device.device_type = kDLCUDA;
    EXPECT_EQ(ovphysx_read_contact_net_friction_forces(m_handle, cb, &dst).status,
              OVPHYSX_API_DEVICE_MISMATCH);
    dst.device.device_type = kDLCPU;

    dst.ndim = 3;
    dst.shape = matrixShape;
    dst.data = matrix;
    ASSERT_EQ(ovphysx_read_contact_normal_force_matrix(m_handle, cb, &dst).status, OVPHYSX_API_SUCCESS);
    for (int c = 0; c < 3; ++c)
        EXPECT_FLOAT_EQ(matrix[c], normal[c]);

    dst.data = matrix;
    ASSERT_EQ(ovphysx_read_contact_friction_force_matrix(m_handle, cb, &dst).status, OVPHYSX_API_SUCCESS);
    for (int c = 0; c < 3; ++c)
        EXPECT_FLOAT_EQ(matrix[c], friction[c]);
    EXPECT_EQ(ovphysx_read_contact_friction_force_matrix(m_handle, cb, nullptr).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    dst.dtype.code = kDLInt;
    EXPECT_EQ(ovphysx_read_contact_friction_force_matrix(m_handle, cb, &dst).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    dst.dtype.code = kDLFloat;
    matrixShape[2] = 2;
    EXPECT_EQ(ovphysx_read_contact_friction_force_matrix(m_handle, cb, &dst).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    matrixShape[2] = 3;
    dst.device.device_type = kDLCUDA;
    EXPECT_EQ(ovphysx_read_contact_friction_force_matrix(m_handle, cb, &dst).status,
              OVPHYSX_API_DEVICE_MISMATCH);
    dst.device.device_type = kDLCPU;

    ASSERT_EQ(ovphysx_destroy_contact_binding(m_handle, cb).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_read_contact_friction_force_matrix(m_handle, cb, &dst).status, OVPHYSX_API_NOT_FOUND);
    dst.ndim = 2;
    dst.shape = netShape;
    EXPECT_EQ(ovphysx_read_contact_net_friction_forces(m_handle, cb, &dst).status, OVPHYSX_API_NOT_FOUND);
}


// Compatibility only: these cases retire with the deprecated contact aliases.
TEST_F(ContactBindingTest, DeprecatedAggregateContactAliases)
{
    ovphysx_usd_handle_t usd = 0;
    ASSERT_TRUE(load_usd_cb(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd));
    ovphysx_string_t sensor = make_ovx_string("/World/Cube1");
    ovphysx_string_t filter = make_ovx_string("/World/BigBase");
    ovphysx_contact_binding_handle_t binding = 0;
    ASSERT_EQ(ovphysx_create_contact_binding(m_handle, &sensor, 1, &filter, 1, 0, &binding).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(step_cb(m_handle, 1.0f / 60.0f));

    float current[3]{}, deprecated[3]{};
    int64_t shape[3]{1, 3, 3};
    DLTensor dst = make_contact_tensor(current, shape, kDLFloat);
    ASSERT_EQ(ovphysx_read_contact_net_normal_forces(m_handle, binding, &dst).status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(current[2], 0.0f);
    dst.data = deprecated;
    ASSERT_EQ(ovphysx_read_contact_net_forces(m_handle, binding, &dst).status, OVPHYSX_API_SUCCESS);
    for (int c = 0; c < 3; ++c)
        EXPECT_FLOAT_EQ(deprecated[c], current[c]);

    dst.ndim = 3;
    shape[1] = 1;
    dst.data = current;
    ASSERT_EQ(ovphysx_read_contact_normal_force_matrix(m_handle, binding, &dst).status, OVPHYSX_API_SUCCESS);
    dst.data = deprecated;
    ASSERT_EQ(ovphysx_read_contact_force_matrix(m_handle, binding, &dst).status, OVPHYSX_API_SUCCESS);
    for (int c = 0; c < 3; ++c)
        EXPECT_FLOAT_EQ(deprecated[c], current[c]);
    struct ReadCase
    {
        decltype(&ovphysx_read_contact_net_normal_forces) read;
        const char* operation;
    };
    const ReadCase reads[] = {
        {ovphysx_read_contact_net_normal_forces, "read_contact_net_normal_forces"},
        {ovphysx_read_contact_net_friction_forces, "read_contact_net_friction_forces"},
        {ovphysx_read_contact_normal_force_matrix, "read_contact_normal_force_matrix"},
        {ovphysx_read_contact_friction_force_matrix, "read_contact_friction_force_matrix"},
        {ovphysx_read_contact_net_forces, "read_contact_net_forces"},
        {ovphysx_read_contact_force_matrix, "read_contact_force_matrix"},
    };
    dst.dtype.code = kDLInt;
    for (const ReadCase& read : reads)
    {
        SCOPED_TRACE(read.operation);
        EXPECT_EQ(read.read(m_handle, binding, &dst).status, OVPHYSX_API_INVALID_ARGUMENT);
        const ovphysx_string_t error = ovphysx_get_last_error();
        EXPECT_EQ(std::string(error.ptr, error.length), std::string(read.operation) + ": expected float32 tensor");
    }
    EXPECT_EQ(ovphysx_destroy_contact_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

TEST_F(ContactBindingTest, DeprecatedDetailedContactAliases)
{
    ovphysx_usd_handle_t usd = 0;
    ASSERT_TRUE(load_usd_cb(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd));
    ovphysx_string_t sensor = make_ovx_string("/World/Cube1");
    ovphysx_string_t filter = make_ovx_string("/World/BigBase");
    ovphysx_contact_binding_handle_t complete = 0, truncated = 0;
    ASSERT_EQ(ovphysx_create_contact_binding(m_handle, &sensor, 1, &filter, 1, 64, &complete).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_create_contact_binding(m_handle, &sensor, 1, &filter, 1, 1, &truncated).status,
              OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 120; ++i)
        ASSERT_TRUE(step_cb(m_handle, 1.0f / 60.0f));

    for (uint32_t capacity : {1u, 64u})
    {
        SCOPED_TRACE(capacity);
        const ovphysx_contact_binding_handle_t binding = capacity == 1 ? truncated : complete;
        const ContactDataBuffers normal = read_normal_contact_buffers(
            m_handle, binding, capacity, 1, 1, ovphysx_read_normal_contact_data);
        const ContactDataBuffers oldNormal = read_normal_contact_buffers(
            m_handle, binding, capacity, 1, 1, ovphysx_read_contact_data);
        ASSERT_GT(normal.required, 1u);
        EXPECT_EQ(normal.status, capacity == 1 ? OVPHYSX_API_BUFFER_TOO_SMALL : OVPHYSX_API_SUCCESS);
        EXPECT_EQ(oldNormal.status, normal.status);
        EXPECT_EQ(oldNormal.required, normal.required);
        EXPECT_EQ(oldNormal.forces, normal.forces);
        EXPECT_EQ(oldNormal.points, normal.points);
        EXPECT_EQ(oldNormal.normals, normal.normals);
        EXPECT_EQ(oldNormal.separations, normal.separations);
        EXPECT_EQ(oldNormal.counts, normal.counts);
        EXPECT_EQ(oldNormal.starts, normal.starts);

        const ContactDataBuffers friction = read_friction_contact_buffers(
            m_handle, binding, capacity, 1, 1, ovphysx_read_friction_contact_data);
        const ContactDataBuffers oldFriction = read_friction_contact_buffers(
            m_handle, binding, capacity, 1, 1, ovphysx_read_friction_data);
        ASSERT_GT(friction.required, 1u);
        EXPECT_EQ(friction.status, capacity == 1 ? OVPHYSX_API_BUFFER_TOO_SMALL : OVPHYSX_API_SUCCESS);
        EXPECT_EQ(oldFriction.status, friction.status);
        EXPECT_EQ(oldFriction.required, friction.required);
        EXPECT_EQ(oldFriction.forces, friction.forces);
        EXPECT_EQ(oldFriction.points, friction.points);
        EXPECT_EQ(oldFriction.counts, friction.counts);
        EXPECT_EQ(oldFriction.starts, friction.starts);
    }
    // Invalid detailed buffers must name the entry point the caller actually used.
    float invalidData[3]{};
    int64_t invalidShape[2]{64, 1};
    DLTensor invalid = make_contact_tensor(invalidData, invalidShape, kDLInt);
    uint32_t required = 0;
    for (bool deprecated : {false, true})
    {
        const decltype(&ovphysx_read_normal_contact_data) readNormal = deprecated ? ovphysx_read_contact_data : ovphysx_read_normal_contact_data;
        const decltype(&ovphysx_read_friction_contact_data) readFriction = deprecated ? ovphysx_read_friction_data : ovphysx_read_friction_contact_data;
        const char* normalOp = deprecated ? "read_contact_data" : "read_normal_contact_data";
        const char* frictionOp = deprecated ? "read_friction_data" : "read_friction_contact_data";
        EXPECT_EQ(readNormal(m_handle, complete, &invalid, nullptr, nullptr, nullptr, nullptr, nullptr, &required).status,
                  OVPHYSX_API_INVALID_ARGUMENT);
        const ovphysx_string_t normalError = ovphysx_get_last_error();
        EXPECT_EQ(std::string(normalError.ptr, normalError.length), std::string(normalOp) + ": expected float32 tensor");
        EXPECT_EQ(readFriction(m_handle, complete, &invalid, nullptr, nullptr, nullptr, &required).status,
                  OVPHYSX_API_INVALID_ARGUMENT);
        const ovphysx_string_t frictionError = ovphysx_get_last_error();
        EXPECT_EQ(std::string(frictionError.ptr, frictionError.length), std::string(frictionOp) + ": expected float32 tensor");
    }
    EXPECT_EQ(ovphysx_destroy_contact_binding(m_handle, truncated).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_contact_binding(m_handle, complete).status, OVPHYSX_API_SUCCESS);
}
