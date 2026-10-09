// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-THREADS-001
 * @covers AC-1 AC-2
 */

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "test_utilities.h"
#include <PxScene.h>
#include <task/PxCpuDispatcher.h>
#include <gtest/gtest.h>
#include <carb/thread/Util.h>

namespace
{
class SolverThreadCount : public ::testing::Test
{
protected:
    ovphysx_handle_t m_handle = OVPHYSX_INVALID_HANDLE;

    void TearDown() override
    {
        if (m_handle != OVPHYSX_INVALID_HANDLE)
        {
            ASSERT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
            EXPECT_EQ(ovphysx_destroy_instance(m_handle).status, OVPHYSX_API_SUCCESS);
        }
    }

    void verifyCount(int32_t requested)
    {
        if (carb::thread::hardware_concurrency() < static_cast<uint32_t>(requested))
            GTEST_SKIP() << "Requested count exceeds this host's default tasking capacity";
        // test_cpp.cmake runs each case alone; test_main owns initialize/shutdown.
        ASSERT_EQ(ovphysx_set_cpu_mode(true).status, OVPHYSX_API_SUCCESS);
        const ovphysx_config_entry_t entry = ovphysx_config_entry_num_threads(requested);
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        args.config_entries = &entry;
        args.config_entry_count = 1;
        ASSERT_EQ(ovphysx_create_instance(&args, &m_handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(
            m_handle, "tests/data/boxes_falling_on_groundplane.usda"));

        void* rawScene = nullptr;
        ASSERT_EQ(ovphysx_get_physx_ptr(m_handle, OVPHYSX_LITERAL("/World/physicsScene"),
            OVPHYSX_PHYSX_TYPE_SCENE, &rawScene).status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(rawScene, nullptr);
        physx::PxScene* scene = static_cast<physx::PxScene*>(rawScene);
        ASSERT_NE(scene->getCpuDispatcher(), nullptr);
        EXPECT_EQ(scene->getCpuDispatcher()->getWorkerCount(), static_cast<uint32_t>(requested));

        // Populate a later ordinal with a new body to exercise input application.
        ovstage_instance_t* stage = test_utils::ovstage_test_attachments()[m_handle].front().stage;
        const char addition[] =
            "#usda 1.0\n(defaultPrim = \"InputBox\")\n"
            "def Cube \"InputBox\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsCollisionAPI\"]) {\n"
            " double size = 1\n vector3f physics:velocity = (1,0,0)\n"
            " double3 xformOp:translate = (0,0,10)\n"
            " uniform token[] xformOpOrder = [\"xformOp:translate\"]\n}\n";
        ovstage_population_usd_reference_handle_t reference{};
        const ovstage_population_enqueue_result_t added = ovstage_population_add_usd_reference_from_string(
            stage, {addition, sizeof(addition) - 1}, {"/World/InputBox", 15}, &reference);
        ASSERT_EQ(added.status, OVSTAGE_OK);
        ovstage_population_op_wait_result_t populationWait{};
        ASSERT_EQ(ovstage_population_wait_op(stage, added.op_index, 10000000000ULL, &populationWait), OVSTAGE_OK);
        const ovstage_population_enqueue_result_t applied = ovstage_population_apply_usd_changes(stage, 2);
        ASSERT_EQ(applied.status, OVSTAGE_OK);
        ASSERT_EQ(ovstage_population_wait_op(stage, applied.op_index, 10000000000ULL, &populationWait), OVSTAGE_OK);
        ovstage_write_floor_desc_t floorDesc{};
        floorDesc.ordinal = 2;
        floorDesc.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floorDesc);
        ASSERT_EQ(floor.status, OVSTAGE_OK);
        ovstage_op_wait_result_t floorWait{};
        ASSERT_EQ(ovstage_wait_op(stage, floor.op_index, 10000000000ULL, &floorWait), OVSTAGE_OK);
        ASSERT_EQ(floorWait.error_op_id_count, 0u);
        ASSERT_EQ(ovstage_release_op(stage, floor.op_index), OVSTAGE_OK);
        ovstage_ordinal_range_t range{};
        range.end_ordinal = 2;
        ASSERT_EQ(ovphysx_update_from_ovstage(m_handle, range).status, OVPHYSX_API_SUCCESS);
        void* actor = nullptr;
        ASSERT_EQ(ovphysx_get_physx_ptr(m_handle, OVPHYSX_LITERAL("/World/InputBox"),
            OVPHYSX_PHYSX_TYPE_ACTOR, &actor).status, OVPHYSX_API_SUCCESS);
        ASSERT_NE(actor, nullptr);
        ASSERT_EQ(ovphysx_step_n_sync(m_handle, 10, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(scene->getCpuDispatcher()->getWorkerCount(), static_cast<uint32_t>(requested));

        // A round trip alone cannot detect two APIs writing the same wrong key.
        // Write the runtime's canonical key independently, then read the typed key.
        ASSERT_EQ(ovphysx_set_global_config(ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/persistent/physics/numThreads"), OVPHYSX_LITERAL("3"))).status,
            OVPHYSX_API_SUCCESS);
        int32_t configured = 0;
        ASSERT_EQ(ovphysx_get_global_config_int32(OVPHYSX_CONFIG_NUM_THREADS, &configured).status,
            OVPHYSX_API_SUCCESS);
        EXPECT_EQ(configured, 3);
    }
};

TEST_F(SolverThreadCount, Workers1) { verifyCount(1); }
TEST_F(SolverThreadCount, Workers2) { verifyCount(2); }
TEST_F(SolverThreadCount, Workers4) { verifyCount(4); }
TEST_F(SolverThreadCount, Workers8) { verifyCount(8); }
} // namespace
