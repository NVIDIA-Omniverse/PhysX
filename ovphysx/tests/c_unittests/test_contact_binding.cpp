// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// PARTIALLY DEPRECATED (tensor-binding-deprecation): the binding-isolation foil retires with the binding.

// Tests for the Contact Binding C ABI:
//   ovphysx_create_contact_binding   - null-argument error conditions
//   ovphysx_destroy_contact_binding  - invalid-handle rejection
//   ovphysx_get_contact_binding_spec - invalid-handle rejection
//   ovphysx_contact_binding_get_sensor_paths - invalid-handle rejection
//   ovphysx_contact_binding_get_filter_paths - invalid-handle rejection

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
 * @implements REQ-CAPI-STRING-001
 * @covers AC-3
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "global_test_environment.h"
#include "test_utilities.h"

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

    r = ovphysx_read_contact_data(
        m_handle, cb, contact_forces, positions, normals, separations, counts, starts);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT);

    r = ovphysx_read_friction_data(
        m_handle, cb, friction_forces, friction_points, counts, starts);
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

// Raw contact data is the filter-less variant of read_contact_data: per-sensor
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

    r = ovphysx_read_raw_contact_data(
        m_handle, cb, &force_t, &point_t, &normal_t, &separation_t, &layout_t, &ids_t);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS) << ovphysx_get_last_error().ptr;

    // At least one sensor must have at least one contact after settling.
    int32_t total_contacts = 0;
    for (int32_t i = 0; i < sensor_count; ++i) total_contacts += layout_buf[i * 2 + 0];
    EXPECT_GT(total_contacts, 0) << "expected at least one contact after 60 steps";

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

    ovphysx_destroy_contact_binding(m_handle, cb);
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

    ovphysx_result_t r = ovphysx_read_raw_contact_data(
        m_handle, cb, &force_t, &point_t, &normal_t, &separation_t, &layout_t, &ids_t2);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "expected shape-mismatch rejection (sensor layout was [S, 4], raw read wants [S, 2])";

    ovphysx_destroy_contact_binding(m_handle, cb);
}
