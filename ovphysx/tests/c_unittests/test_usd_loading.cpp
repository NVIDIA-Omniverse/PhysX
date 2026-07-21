// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// ovstage stage-ingest tests.
//
// ovphysx no longer exposes a public direct USD loader. Tests that need scene
// data populate an ovstage instance from USD, attach that caller-owned stage,
// and explicitly drain the committed ordinal range through
// ovphysx_update_from_ovstage().
//
// EXPECTED WARNINGS/ERRORS DURING TEST EXECUTION:
//
// 1. USD Stage Errors (from the non-existent-file test):
//    - "Runtime Error: Failed to open layer @tests/data/nonexistent.usda@"
//    - "[Error] [omni.physx] PhysX could not find USD stage"
//    These are intentional - the test verifies error handling for invalid files.
//
// 2. TfType Redefinition Warnings (may appear across multiple tests):
//    - "Coding Error: TfType 'omni::fabric::AttributeValuesChangedNotice' already has a defined C++ type"
//    This is a known USD/Fabric library behavior when types are registered multiple times
//    across different test fixtures. It does not indicate a problem with the tests.

#include <gtest/gtest.h>

#include "global_test_environment.h"
#include "ovphysx/ovphysx.h"
#include "test_utilities.h"

using namespace test_utils;

TEST_F(PhysXTestFixture, AttachOvstageMinimalScene)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/minimal_scene.usda"));
}

TEST_F(PhysXTestFixture, AttachOvstagePhysicsScene)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/simple_physics_scene.usda"));
}

// Loading a non-existent file must fail. ovphysx no longer exposes the public
// direct USD loader, so this goes through the ovstage attach path.
//
// Expected log noise (intentional, from the failed open attempt):
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
