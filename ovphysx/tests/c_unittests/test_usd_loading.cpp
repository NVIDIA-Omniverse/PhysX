// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// ovstage stage-ingest tests.
//
// ovphysx has no public direct USD loader. Tests that need scene data populate
// an ovstage instance from USD, attach that caller-owned stage, and explicitly
// drain the committed ordinal range through ovphysx_update_from_ovstage().
//
// Expected warnings and errors during test execution:
//
// 1. USD stage errors from the non-existent-file test, which verifies the error
//    handling for invalid files:
//    - "Runtime Error: Failed to open layer @tests/data/nonexistent.usda@"
//    - "[Error] [omni.physx] PhysX could not find USD stage"
//
// 2. TfType redefinition warnings, which may appear across multiple tests:
//    - "Coding Error: TfType 'omni::fabric::AttributeValuesChangedNotice' already has a defined C++ type"
//    USD/Fabric emits this when types are registered multiple times across
//    different test fixtures. It does not indicate a problem with the tests.

/**
 * @implements REQ-CAPI-OVSTAGE-ATTACH-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-CAPI-DETACH-002
 * @covers AC-4
 */

#include <gtest/gtest.h>
#include <ovstage/ovx_path_dictionary.h>

#include "global_test_environment.h"
#include "ovphysx/ovphysx.h"
#include "test_utilities.h"

#include <cstring>

using namespace test_utils;

namespace
{

bool waitAndReleaseOvstageOp(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue)
{
    if (enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_INVALID_OP_ID)
        return false;

    ovstage_op_wait_result_t waitResult{};
    const ovstage_api_status_t waitStatus =
        ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &waitResult);
    if (waitStatus != OVSTAGE_OK && waitStatus != OVSTAGE_ERROR_OP_FAILED)
        return false;
    return ovstage_release_op(stage, enqueue.op_index) == OVSTAGE_OK &&
           waitStatus == OVSTAGE_OK && waitResult.error_op_id_count == 0;
}

bool populateTwoArticulations(ovstage_instance_t* stage)
{
    if (!register_physx_schemas_with_ovstage())
        return false;
    constexpr const char* path = "tests/data/two_articulations.usda";
    const ovx_string_t pathString{ path, std::strlen(path) };
    const ovstage_population_enqueue_result_t enqueue = ovstage_population_open_usd_from_file(
        stage, pathString, /*ordinal=*/1, /*time=*/0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS);
    if (enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_POPULATION_INVALID_OP_ID)
        return false;

    ovstage_population_op_wait_result_t waitResult{};
    const ovstage_api_status_t waitStatus =
        ovstage_population_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &waitResult);
    return waitStatus == OVSTAGE_OK && waitResult.error_op_id_count == 0;
}

bool writeUnconsumedAttribute(ovstage_instance_t* stage, ovx_token_t& attributeToken)
{
    ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
    if (!dictionary)
        return false;

    constexpr const char* path = "/World/articulation";
    const ovx_string_t pathString{ path, std::strlen(path) };
    ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
    const ovx_api_status_t listStatus =
        ovx_path_dictionary_create_path_list_from_strings(dictionary, &pathString, 1, &pathList);
    if (listStatus != OVX_OK)
        return false;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_api_status_t queryStatus = ovstage_query_from_path_list(stage, pathList, &query);
    (void)ovx_path_dictionary_destroy_path_list(dictionary, pathList);
    if (queryStatus != OVSTAGE_OK)
        return false;

    float value = 42.0f;
    int64_t shape[1]{ 1 };
    DLTensor tensor{};
    tensor.data = &value;
    tensor.device = { kDLCPU, 0 };
    tensor.ndim = 1;
    tensor.dtype = { kDLFloat, 32, 1 };
    tensor.shape = shape;

    ovstage_write_data_t write{};
    write.tensors = &tensor;
    write.tensor_count = 1;

    constexpr const char* attribute = "custom:unconsumed";
    ovx_string_or_token_t name{};
    name.string = { attribute, std::strlen(attribute) };
    const bool writeResult = waitAndReleaseOvstageOp(
        stage, ovstage_write_attribute(stage, query, name, /*ordinal=*/1, write, OVSTAGE_PRIM_MODE_UPSERT));
    const bool releaseResult = waitAndReleaseOvstageOp(stage, ovstage_release_query(stage, query));
    const ovx_string_t attributeName{ attribute, std::strlen(attribute) };
    const ovx_api_status_t tokenStatus =
        ovx_path_dictionary_intern_token(dictionary, attributeName, &attributeToken);
    return writeResult && releaseResult && tokenStatus == OVX_OK && attributeToken != OVX_INVALID_TOKEN;
}

bool advanceWriteFloor(ovstage_instance_t* stage,
                       ovstage_scope_t scope,
                       const ovx_token_t* attributes = nullptr,
                       size_t attributeCount = 0)
{
    ovstage_write_floor_desc_t floor{};
    floor.ordinal = 1;
    floor.scope = scope;
    floor.attributes = attributes;
    floor.attribute_count = attributeCount;
    return waitAndReleaseOvstageOp(stage, ovstage_advance_write_floor(stage, &floor));
}

struct OvstageGuard
{
    ~OvstageGuard()
    {
        if (attachedHandle != OVPHYSX_INVALID_HANDLE)
        {
            if (ovphysx_detach_ovstage(attachedHandle).status != OVPHYSX_API_SUCCESS)
            {
                ADD_FAILURE() << "failed to detach caller-owned ovstage during cleanup";
                return;
            }
        }
        ovstage_destroy_instance(stage);
    }

    ovstage_instance_t* stage = nullptr;
    ovphysx_handle_t attachedHandle = OVPHYSX_INVALID_HANDLE;
};

} // namespace

TEST_F(PhysXTestFixture, AttachOvstageMinimalScene)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));
}

TEST_F(PhysXTestFixture, AttachOvstagePhysicsScene)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/simple_physics_scene.usda"));
}

// Loading a non-existent file must fail. ovphysx has no public direct USD
// loader, so this goes through the ovstage attach path.
//
// Expected log noise from the failed open attempt:
//   "Runtime Error: Failed to open layer @tests/data/nonexistent.usda@"
//   "[Error] [omni.physx] PhysX could not find USD stage"
TEST_F(PhysXTestFixture, AttachOvstageNonExistentFileFails)
{
    EXPECT_FALSE(attach_usd_with_ovstage(m_handle, "tests/data/nonexistent.usda"));
}

TEST_F(PhysXTestFixture, AttachOvstageTwiceRejected)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));
    EXPECT_FALSE(attach_usd_with_ovstage(m_handle, "tests/data/simple_physics_scene.usda"));
}

TEST_F(PhysXTestFixture, DetachAllowsOvstageReattach)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));
    destroy_ovstage_test_attachments(m_handle);

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/simple_physics_scene.usda"));
}

TEST_F(PhysXTestFixture, ResetDetachesOvstage)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));

    ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
    if (reset.op_index != 0)
    {
        ASSERT_TRUE(waitForOperationSuccess(m_handle, reset.op_index, 3'000'000'000ULL));
    }
    destroy_ovstage_test_attachments(m_handle);

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/simple_physics_scene.usda"));
}

TEST_F(PhysXTestFixture, UnsealedAttachFailsAndSealedRetrySucceeds)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-unsealed-attach-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };

    ASSERT_TRUE(populateTwoArticulations(stage));

    const ovphysx_result_t unsealedAttach = ovphysx_attach_ovstage(m_handle, stage, 1);
    if (unsealedAttach.status == OVPHYSX_API_SUCCESS)
    {
        stageGuard.attachedHandle = m_handle;
        FAIL() << "unsealed ovstage attachment unexpectedly succeeded";
    }
    ASSERT_EQ(unsealedAttach.status, OVPHYSX_API_ERROR);
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_ALL));
    ASSERT_EQ(ovphysx_attach_ovstage(m_handle, stage, 1).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;

    ASSERT_EQ(ovphysx_detach_ovstage(m_handle).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = OVPHYSX_INVALID_HANDLE;
}

TEST_F(PhysXTestFixture, ScopedSealAllowsUnrelatedUnsealedData)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-scoped-seal-attach-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };

    ASSERT_TRUE(populateTwoArticulations(stage));

    ovx_token_t unrelatedAttribute = OVX_INVALID_TOKEN;
    ASSERT_TRUE(writeUnconsumedAttribute(stage, unrelatedAttribute));
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_EXCLUDE, &unrelatedAttribute, 1));

    ASSERT_EQ(ovphysx_attach_ovstage(m_handle, stage, 1).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;

    ASSERT_EQ(ovphysx_detach_ovstage(m_handle).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = OVPHYSX_INVALID_HANDLE;
}

// Articulations parsed at attach time stay in the runtime's pending list until the
// first step hands them to a scene. Detaching before ever stepping must drop them,
// otherwise the next attach's first step indexes the dead attach's records.
TEST_F(PhysXTestFixture, DetachWithoutStepDropsPendingArticulations)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-detach-pending-articulations-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };

    ASSERT_TRUE(populateTwoArticulations(stage));
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_ALL));
    ASSERT_EQ(ovphysx_attach_ovstage(m_handle, stage, 1).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;

    // No ovphysx_step() before the detach, so nothing drains the pending list but teardown.
    ASSERT_EQ(ovphysx_detach_ovstage(m_handle).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = OVPHYSX_INVALID_HANDLE;

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));

    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForOperationSuccess(m_handle, step.op_index, 10'000'000'000ULL));
}

TEST_F(PhysXTestFixture, AttachOvstageRejectsZeroReadOrdinal)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-zero-read-ordinal-attach-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };

    const ovphysx_result_t zeroOrdinal = ovphysx_attach_ovstage(m_handle, stage, 0);
    ASSERT_EQ(zeroOrdinal.status, OVPHYSX_API_INVALID_ARGUMENT);

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));
}
