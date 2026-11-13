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
 * @implements REQ-CAPI-OVSTAGE-UPDATE-001
 * @covers AC-1 AC-3 AC-4
 *
 * @implements REQ-CAPI-OVSTAGE-UPDATE-002
 * @covers AC-1
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
#include <string>

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
    const ovstage_population_enqueue_result_t enqueue = ovphysx_sample_open_usd_from_file(
        stage, pathString, /*ordinal=*/1, /*time=*/0.0, /*with_rendering=*/false);
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
    ovx_token_or_string_t name{};
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
                       size_t attributeCount = 0,
                       uint64_t ordinal = 1)
{
    ovstage_write_floor_desc_t floor{};
    floor.ordinal = ordinal;
    floor.scope = scope;
    floor.attributes = attributes;
    floor.attribute_count = attributeCount;
    return waitAndReleaseOvstageOp(stage, ovstage_advance_write_floor(stage, &floor));
}

void expectArticulations(ovphysx_instance_handle_t handle, uint64_t count = 2)
{
    ovphysx_query_handle_t query = 0;
    ovstage_query_result_t summary{};
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_ARTICULATION,
                                                          OVPHYSX_SCOPE_ALL, &query, &summary)).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(summary.total_prim_count, count);
    if (query != 0)
    {
        EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status, OVPHYSX_API_SUCCESS);
    }
}

struct OvstageGuard
{
    ~OvstageGuard()
    {
        if (attachedHandle != OVPHYSX_INVALID_HANDLE)
        {
            if (test_utils::complete_task(attachedHandle, ovphysx_detach_ovstage(attachedHandle)).status != OVPHYSX_API_SUCCESS)
            {
                ADD_FAILURE() << "failed to detach caller-owned ovstage during cleanup";
                return;
            }
        }
        ovstage_destroy_instance(stage);
    }

    ovstage_instance_t* stage = nullptr;
    ovphysx_instance_handle_t attachedHandle = OVPHYSX_INVALID_HANDLE;
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

TEST_F(PhysXTestFixture, UnsealedAttachFailsAndSealedRetrySucceeds)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-unsealed-attach-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };

    ASSERT_TRUE(populateTwoArticulations(stage));

    const ovphysx_enqueue_result_t unsealedAttach = ovphysx_attach_ovstage(m_handle, stage, 1);
    ASSERT_EQ(unsealedAttach.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(unsealedAttach.op_index, 0u);
    ASSERT_EQ(unsealedAttach.instance, m_handle);
    stageGuard.attachedHandle = m_handle;
    ovphysx::physx::WaitResult unsealedResult;
    ASSERT_EQ(unsealedResult.waitTask(m_handle, unsealedAttach), OVPHYSX_API_ERROR);
    ASSERT_EQ(unsealedResult.errorCount(), 1u);
    EXPECT_EQ(unsealedResult.errorTaskAt(0).op_index, unsealedAttach.op_index);
    const ovphysx_string_t error = ovphysx_get_last_task_error(m_handle, unsealedAttach);
    ASSERT_NE(error.ptr, nullptr);
    const std::string message(error.ptr, error.length);
    EXPECT_NE(message.find("schema scan"), std::string::npos) << message;
    EXPECT_NE(message.find("ovstage_read_attributes"), std::string::npos) << message;
    EXPECT_NE(message.find("ordinal 1"), std::string::npos) << message;
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_ALL));
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_attach_ovstage(m_handle, stage, 1)).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;
    EXPECT_EQ(ovphysx_get_last_error().length, 0u);

    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = OVPHYSX_INVALID_HANDLE;
}

class PartialSealAttachTest : public PhysXTestFixture, public ::testing::WithParamInterface<bool>
{
};

TEST_P(PartialSealAttachTest, FailedAttachCanRetryAfterSealingJointRelationships)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-partial-seal-attach-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };
    ASSERT_TRUE(populateTwoArticulations(stage));

    ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
    ASSERT_NE(dictionary, nullptr);
    const bool schemasOnly = GetParam();
    const char* names[] = { schemasOnly ? "usd-schemas" : "physics:body0", "physics:body1" };
    const size_t attributeCount = schemasOnly ? 1 : 2;
    ovx_token_t attributes[2]{};
    for (size_t i = 0; i < attributeCount; ++i)
    {
        const ovx_string_t name{ names[i], std::strlen(names[i]) };
        ASSERT_EQ(ovx_path_dictionary_intern_token(dictionary, name, &attributes[i]), OVX_OK);
    }
    ASSERT_TRUE(advanceWriteFloor(stage, schemasOnly ? OVSTAGE_SCOPE_INCLUDE : OVSTAGE_SCOPE_EXCLUDE,
                                 attributes, attributeCount));

    const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(m_handle, stage, 1);
    ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(attach.op_index, 0u);
    stageGuard.attachedHandle = m_handle;
    ovphysx::physx::WaitResult result;
    ASSERT_EQ(result.waitTask(m_handle, attach), OVPHYSX_API_ERROR);
    ASSERT_EQ(result.errorCount(), 1u);
    EXPECT_EQ(result.errorTaskAt(0).op_index, attach.op_index);
    const ovphysx_string_t error = ovphysx_get_last_task_error(m_handle, attach);
    ASSERT_NE(error.ptr, nullptr);
    const std::string message(error.ptr, error.length);
    EXPECT_NE(message.find("ordinal 1"), std::string::npos) << message;
    if (!schemasOnly)
    {
        EXPECT_NE(message.find("relationship read"), std::string::npos) << message;
        EXPECT_NE(message.find("physics:body"), std::string::npos) << message;
        EXPECT_NE(message.find("/World/"), std::string::npos) << message;
    }
    bool attached = true;
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, nullptr, &attached).status, OVPHYSX_API_SUCCESS);
    ASSERT_FALSE(attached);
    stageGuard.attachedHandle = OVPHYSX_INVALID_HANDLE;

    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_ALL));
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_attach_ovstage(m_handle, stage, 1)).status,
              OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;
    expectArticulations(m_handle);
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = OVPHYSX_INVALID_HANDLE;
}

INSTANTIATE_TEST_SUITE_P(PartialSeals, PartialSealAttachTest, ::testing::Values(true, false));

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

    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_attach_ovstage(m_handle, stage, 1)).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;
    expectArticulations(m_handle);

    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
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
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_attach_ovstage(m_handle, stage, 1)).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;

    // No ovphysx_step() before the detach, so nothing drains the pending list but teardown.
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = OVPHYSX_INVALID_HANDLE;

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));

    const ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
    ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(m_handle, step, 10'000'000'000ULL));
}

// @implements REQ-CAPI-OVSTAGE-ATTACH-001
// @covers AC-5
// @implements REQ-CAPI-OVSTAGE-UPDATE-001
// @covers AC-3
// @implements REQ-CAPI-ASYNC-003
// @covers AC-3
TEST_F(PhysXTestFixture, AttachAndUpdateReturnCollectableTasks)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-lifecycle-task-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{stage};
    ASSERT_TRUE(populateTwoArticulations(stage));
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_ALL));

    const ovphysx_enqueue_result_t attach = ovphysx_attach_ovstage(m_handle, stage, 1);
    ASSERT_EQ(attach.status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(attach.op_index, 0u);
    EXPECT_EQ(attach.instance, m_handle);
    stageGuard.attachedHandle = m_handle;
    ASSERT_TRUE(waitForTaskSuccess(m_handle, attach));

    // The already-attached check runs in the attach job: the duplicate is accepted and its
    // task fails, and the first attachment stays live.
    const ovphysx_enqueue_result_t duplicate = ovphysx_attach_ovstage(m_handle, stage, 1);
    ASSERT_EQ(duplicate.status, OVPHYSX_API_SUCCESS);
    EXPECT_GT(duplicate.op_index, attach.op_index);
    EXPECT_EQ(duplicate.instance, m_handle);
    EXPECT_TRUE(waitForTaskFailure(m_handle, duplicate, "a stage is already attached"));

    const ovstage_ordinal_range_t range{1, 1, true};
    const ovphysx_enqueue_result_t update = ovphysx_update_from_ovstage(m_handle, range);
    ASSERT_EQ(update.status, OVPHYSX_API_SUCCESS);
    EXPECT_GT(update.op_index, attach.op_index);
    EXPECT_EQ(update.instance, m_handle);
    bool attached = false;
    ASSERT_EQ(ovphysx_is_stage_attached(m_handle, stage, &attached).status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(attached);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, update));
    ovphysx::physx::WaitResult collected;
    EXPECT_EQ(collected.waitTask(m_handle, attach, 0), OVPHYSX_API_NOT_FOUND);
    EXPECT_EQ(collected.waitTask(m_handle, update, 0), OVPHYSX_API_NOT_FOUND);
}

TEST_F(PhysXTestFixture, AttachOvstageRejectsZeroReadOrdinal)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-zero-read-ordinal-attach-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };

    // 0 is reserved, and the negative ordinals ovstage also writes are not attachable.
    for (const ovstage_ordinal_t ordinal : { ovstage_ordinal_t{ 0 }, ovstage_ordinal_t{ -1 }, OVSTAGE_ORDINAL_MIN })
    {
        const ovphysx_enqueue_result_t rejected = ovphysx_attach_ovstage(m_handle, stage, ordinal);
        ASSERT_EQ(rejected.status, OVPHYSX_API_INVALID_ARGUMENT) << ordinal;
        EXPECT_EQ(rejected.op_index, 0u);
        EXPECT_EQ(rejected.instance, 0u);
    }

    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));
}

TEST_F(PhysXTestFixture, PartialSealIncrementalScanCanRetryAfterSealingJointRelationships)
{
    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-partial-seal-update-test";
    ovstage_instance_t* stage = nullptr;
    ASSERT_EQ(ovstage_create_instance(&desc, &stage), OVSTAGE_OK);
    OvstageGuard stageGuard{ stage };
    ASSERT_TRUE(populateTwoArticulations(stage));
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_ALL));
    ASSERT_EQ(complete_task(m_handle, ovphysx_attach_ovstage(m_handle, stage, 1)).status, OVPHYSX_API_SUCCESS);
    stageGuard.attachedHandle = m_handle;
    expectArticulations(m_handle);

    // Add a new subtree after attachment so its properties are initialized by
    // the structural rescan, rather than the change feed's existing-object reads.
    constexpr const char* usda = R"usda(#usda 1.0
(defaultPrim = "Added")
def Xform "Added" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Cube "Link0" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"])
    {
        double size = 1
    }
    def Cube "Link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"])
    {
        double size = 1
        double3 xformOp:translate = (0, 0, 2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
    def PhysicsRevoluteJoint "Joint"
    {
        rel physics:body0 = </Added/Link0>
        rel physics:body1 = </Added/Link1>
        uniform token physics:axis = "X"
    }
}
)usda";
    const auto waitPopulation = [stage](ovstage_population_enqueue_result_t enqueue) {
        if (enqueue.status != OVSTAGE_OK || enqueue.op_index == OVSTAGE_POPULATION_INVALID_OP_ID)
            return false;
        ovstage_population_op_wait_result_t waitResult{};
        return ovstage_population_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &waitResult) == OVSTAGE_OK &&
               waitResult.error_op_id_count == 0;
    };
    ovstage_population_usd_reference_handle_t reference{};
    constexpr const char* target = "/World/Added";
    ASSERT_TRUE(waitPopulation(ovstage_population_add_usd_reference_from_string(
        stage, { usda, std::strlen(usda) }, { target, std::strlen(target) }, &reference)));
    ASSERT_TRUE(waitPopulation(ovstage_population_apply_usd_changes(stage, 2)));

    ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
    ASSERT_NE(dictionary, nullptr);
    const char* names[] = { "physics:body0", "physics:body1" };
    ovx_token_t relationships[2]{};
    for (size_t i = 0; i < 2; ++i)
    {
        ASSERT_EQ(ovx_path_dictionary_intern_token(dictionary, { names[i], std::strlen(names[i]) }, &relationships[i]),
                  OVX_OK);
    }
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_EXCLUDE, relationships, 2, 2));
    ovstage_ordinal_range_t range{};
    range.start_ordinal = 2;
    range.end_ordinal = 2;
    range.has_start_ordinal = true;
    const ovphysx_enqueue_result_t update = ovphysx_update_from_ovstage(m_handle, range);
    ASSERT_EQ(update.status, OVPHYSX_API_SUCCESS);
    ovphysx::physx::WaitResult result;
    EXPECT_EQ(result.waitTask(m_handle, update), OVPHYSX_API_ERROR);
    EXPECT_EQ(result.errorCount(), 1u);
    const ovphysx_string_t error = ovphysx_get_last_task_error(m_handle, update);
    const std::string message(error.ptr ? error.ptr : "", error.length);
    EXPECT_NE(message.find("relationship read"), std::string::npos) << message;
    EXPECT_NE(message.find("ordinal 2"), std::string::npos) << message;
    expectArticulations(m_handle);

    // A second unsealed attempt must also fail, without consuming the range.
    EXPECT_EQ(complete_task(m_handle, ovphysx_update_from_ovstage(m_handle, range)).status, OVPHYSX_API_ERROR);
    expectArticulations(m_handle);
    ASSERT_TRUE(advanceWriteFloor(stage, OVSTAGE_SCOPE_ALL, nullptr, 0, 2));
    ASSERT_EQ(complete_task(m_handle, ovphysx_update_from_ovstage(m_handle, range)).status, OVPHYSX_API_SUCCESS);
    expectArticulations(m_handle, 3);
    ASSERT_EQ(complete_task(m_handle, ovphysx_update_from_ovstage(m_handle, range)).status, OVPHYSX_API_SUCCESS);
    expectArticulations(m_handle, 3);
}
