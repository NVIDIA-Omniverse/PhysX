// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/**
 * Comprehensive C++ Wrapper Test Suite
 * 
 * Tests all aspects of the C++ wrapper including:
 * - Instance lifecycle and RAII
 * - USD loading
 * - Simulation operations
 * - Error handling
 * - Resource management
 */

#include <gtest/gtest.h>
#include "ovphysx/experimental/ovphysx.hpp"
#include "ovphysx/ovphysx_types.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <cassert>
#include <vector>
#include <memory>
#include <cstring>

// Helper Functions
namespace {

std::vector<float> createTestData(size_t count) {
    std::vector<float> data(count);
    for (size_t i = 0; i < count; ++i) {
        data[i] = static_cast<float>(i) * 0.1f;
    }
    return data;
}

} // namespace

// Test fixture for tests that need a global SDK instance
class CppWrapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

        ovphysx_handle_t handle = 0;
        ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        m_sdk = std::make_unique<ovphysx::PhysX>(handle);
        ASSERT_TRUE(*m_sdk);
    }

    void TearDown() override {
        if (m_sdk) {
            test_utils::destroy_ovstage_test_attachments(m_sdk->handle());
        }
        m_sdk.reset();
    }

    std::unique_ptr<ovphysx::PhysX> m_sdk;
};

//------------------------------------------------------------------------------------------------------------
// SDK Construction and Destruction
//------------------------------------------------------------------------------------------------------------

TEST(CppWrapper, SDKConstruction) {
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = 0;
    ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    
    ovphysx::PhysX sdk(handle);

    EXPECT_TRUE(sdk) << "SDK is in valid state";
}

//------------------------------------------------------------------------------------------------------------
// CreateArgs
//------------------------------------------------------------------------------------------------------------

TEST(CppWrapper, CreateArgsDefaultConstruction) {
    ovphysx::CreateArgs args;
    const ovphysx_create_args& c = args.cArgs();

    EXPECT_EQ(c.active_cuda_gpus.ptr, nullptr);
    EXPECT_EQ(c.active_cuda_gpus.length, 0u);
    EXPECT_EQ(c.config_entries, nullptr);
    EXPECT_EQ(c.config_entry_count, 0u);
    EXPECT_EQ(c.bundled_deps_path.ptr, nullptr);
    EXPECT_EQ(c.bundled_deps_path.length, 0u);
}

TEST(CppWrapper, CreateArgsSetters) {
    ovphysx::CreateArgs args;

    args.setActiveCudaGpus("3");
    EXPECT_STREQ(args.cArgs().active_cuda_gpus.ptr, "3");
    EXPECT_EQ(args.cArgs().active_cuda_gpus.length, 1u);

    ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_num_threads(4),
    };
    args.setConfigEntries(entries, 1);
    EXPECT_EQ(args.cArgs().config_entries, entries);
    EXPECT_EQ(args.cArgs().config_entry_count, 1u);

    std::string path = "/some/path";
    args.setBundledDepsPath(path);
    EXPECT_STREQ(args.cArgs().bundled_deps_path.ptr, path.c_str());
    EXPECT_EQ(args.cArgs().bundled_deps_path.length, path.size());
}

TEST(CppWrapper, CreateArgsCreateCpu) {
    ovphysx::CreateArgs args;

    ovphysx::PhysX sdk;
    auto status = ovphysx::PhysX::create(sdk, args);
    ASSERT_EQ(status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(sdk);

    // ovphysx_step() requires an attached stage (rejects stage-less handles --
    // see NVBugs 6433668 MR review: a stage-less step() must never reach
    // physxSim->simulate(), since IPhysxSimulation is a process-wide singleton
    // and that call would silently advance whatever OTHER handle's stage is
    // attached instead).
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(sdk.handle(), "tests/data/simple_physics_scene.usda"));

    status = sdk.step(0.016f);
    EXPECT_EQ(status, OVPHYSX_API_SUCCESS);
    auto wait = sdk.waitAll();
    EXPECT_FALSE(wait.hasErrors());
}

TEST(CppWrapper, CreateArgsWithConfigEntries) {
    ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_num_threads(2),
        ovphysx_config_entry_disable_contact_processing(true),
    };

    ovphysx::CreateArgs args;
    args.setConfigEntries(entries, 2);

    ovphysx::PhysX sdk;
    auto status = ovphysx::PhysX::create(sdk, args);
    ASSERT_EQ(status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(sdk);
}

TEST_F(CppWrapperTest, UpdateArticulationsKinematicRequiresStage) {
    auto status = m_sdk->updateArticulationsKinematic();
    EXPECT_EQ(status, OVPHYSX_API_ERROR);
}

TEST(CppWrapper, CreateArgsValidationRejectsNullConfigEntries) {
    ovphysx::CreateArgs args;
    args.setConfigEntries(nullptr, 5);

    ovphysx::PhysX sdk;
    auto status = ovphysx::PhysX::create(sdk, args);
    EXPECT_EQ(status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_FALSE(sdk);
}

TEST(CppWrapperGpuTest, CreateArgsCreateAuto) {
#if !OVPHYSX_ENABLE_GPU_TESTS
    GTEST_SKIP() << "GPU tests disabled at compile time";
#endif
    ovphysx::CreateArgs args;

    ovphysx::PhysX sdk;
    auto status = ovphysx::PhysX::create(sdk, args);
    if (status == OVPHYSX_API_GPU_NOT_AVAILABLE) {
        GTEST_SKIP() << "GPU not available";
    }
    ASSERT_EQ(status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(sdk);

    // ovphysx_step() requires an attached stage (rejects stage-less handles --
    // see NVBugs 6433668 MR review: a stage-less step() must never reach
    // physxSim->simulate(), since IPhysxSimulation is a process-wide singleton
    // and that call would silently advance whatever OTHER handle's stage is
    // attached instead).
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(sdk.handle(), "tests/data/simple_physics_scene.usda"));

    status = sdk.step(0.016f);
    EXPECT_EQ(status, OVPHYSX_API_SUCCESS);
    auto wait = sdk.waitAll();
    EXPECT_FALSE(wait.hasErrors());
}

TEST(CppWrapperGpuTest, CreateArgsCreateGpu) {
#if !OVPHYSX_ENABLE_GPU_TESTS
    GTEST_SKIP() << "GPU tests disabled at compile time";
#endif
    ovphysx::CreateArgs args;

    ovphysx::PhysX sdk;
    auto status = ovphysx::PhysX::create(sdk, args);
    if (status == OVPHYSX_API_GPU_NOT_AVAILABLE) {
        GTEST_SKIP() << "GPU not available";
    }
    ASSERT_EQ(status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(sdk);

    // ovphysx_step() requires an attached stage (rejects stage-less handles --
    // see NVBugs 6433668 MR review: a stage-less step() must never reach
    // physxSim->simulate(), since IPhysxSimulation is a process-wide singleton
    // and that call would silently advance whatever OTHER handle's stage is
    // attached instead).
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(sdk.handle(), "tests/data/simple_physics_scene.usda"));

    status = sdk.step(0.016f);
    EXPECT_EQ(status, OVPHYSX_API_SUCCESS);
    auto wait = sdk.waitAll();
    EXPECT_FALSE(wait.hasErrors());
}

//------------------------------------------------------------------------------------------------------------
// SDK Lifecycle Management
//------------------------------------------------------------------------------------------------------------

TEST(CppWrapper, SDKLifecycle) {
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

        ovphysx_handle_t handle = 0;
        ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        
        ovphysx::PhysX sdk1(handle);
        EXPECT_TRUE(sdk1);
    }

    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

        ovphysx_handle_t handle = 0;
        ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        
        ovphysx::PhysX sdk2(handle);
        EXPECT_TRUE(sdk2);
    }

    SUCCEED();
}

//------------------------------------------------------------------------------------------------------------
// ovstage attach/update
//------------------------------------------------------------------------------------------------------------

TEST_F(CppWrapperTest, AttachOvstageFromUsd) {
    ASSERT_NE(m_sdk, nullptr);
    ASSERT_TRUE(*m_sdk);

    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_sdk->handle(), "tests/data/simple_physics_scene.usda"));
    SUCCEED() << "ovstage attach/update completed successfully";
}

//------------------------------------------------------------------------------------------------------------
// Simulation Step
//------------------------------------------------------------------------------------------------------------

TEST_F(CppWrapperTest, SimulationStep) {
    ASSERT_NE(m_sdk, nullptr);
    ASSERT_TRUE(*m_sdk);

    // ovphysx_step() requires an attached stage (rejects stage-less handles --
    // see NVBugs 6433668 MR review: a stage-less step() must never reach
    // physxSim->simulate(), since IPhysxSimulation is a process-wide singleton
    // and that call would silently advance whatever OTHER handle's stage is
    // attached instead).
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_sdk->handle(), "tests/data/simple_physics_scene.usda"));

    float dt = 0.016f;

    ovphysx_api_status_t status = m_sdk->step(dt);
    ASSERT_EQ(status, OVPHYSX_API_SUCCESS);
    auto wait_result = m_sdk->waitAll();
    ASSERT_FALSE(wait_result.hasErrors());
    SUCCEED() << "Simulation step completed successfully";
}

//------------------------------------------------------------------------------------------------------------
// Error Handling
//------------------------------------------------------------------------------------------------------------

TEST_F(CppWrapperTest, ErrorHandling) {
    ASSERT_NE(m_sdk, nullptr);
    ASSERT_TRUE(*m_sdk);

    ovphysx_api_status_t result = m_sdk->attachOvstage(nullptr, /*read_ordinal=*/1);
    EXPECT_NE(result, OVPHYSX_API_SUCCESS);
    
    SUCCEED() << "Invalid ovstage handle handled gracefully";
}

//------------------------------------------------------------------------------------------------------------
// Resource Management
//------------------------------------------------------------------------------------------------------------

TEST_F(CppWrapperTest, ResourceManagement) {
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

        ovphysx_handle_t handle = 0;
        ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        
        ovphysx::PhysX sdk(handle);

        sdk.step(0.016f);
        sdk.waitAll();
    }
    
    SUCCEED() << "Resource management (RAII) works correctly";
}

//------------------------------------------------------------------------------------------------------------
// Clone Tests - Focus on C++ wrapper-specific concerns
// Comprehensive clone functionality is tested in the C layer (test_clone.cpp)
// These tests verify the C++ wrapper correctly calls through and handles errors
//------------------------------------------------------------------------------------------------------------

TEST_F(CppWrapperTest, CloneBasicFunctionality) {
    // Smoke test: verify clone works through C++ wrapper
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_sdk->handle(), "tests/data/basic_simulation.usda"));

    // Clone to multiple targets (proves vector<string> marshaling works)
    std::vector<std::string> targets = {
        "/World/envs/env1",
        "/World/envs/env2",
        "/World/envs/env3"
    };
    ASSERT_EQ(m_sdk->clone("/World/envs/env0", targets), OVPHYSX_API_SUCCESS);
    auto wait = m_sdk->waitAll();
    ASSERT_FALSE(wait.hasErrors());

    // Verify simulation still works with clones
    ASSERT_EQ(m_sdk->step(1.0f/60.0f), OVPHYSX_API_SUCCESS);
    wait = m_sdk->waitAll();
    ASSERT_FALSE(wait.hasErrors());
}

TEST_F(CppWrapperTest, CloneErrorHandling) {
    // Verify C++ wrapper converts C errors to false return values correctly

    // Empty source path
    std::vector<std::string> targets = {"/World/envs/env1"};
    EXPECT_EQ(m_sdk->clone("", targets), OVPHYSX_API_INVALID_ARGUMENT) << "Empty source should return error";

    // Empty targets
    std::vector<std::string> empty_targets;
    EXPECT_EQ(m_sdk->clone("/World/envs/env0", empty_targets), OVPHYSX_API_INVALID_ARGUMENT) << "Empty targets should return error";

    // Target matching source
    std::vector<std::string> same_targets = {"/World/envs/env0"};
    EXPECT_EQ(m_sdk->clone("/World/envs/env0", same_targets), OVPHYSX_API_INVALID_ARGUMENT) << "Same source/target should return error";

    // No USD loaded
    EXPECT_EQ(m_sdk->clone("/World/envs/env0", targets), OVPHYSX_API_ERROR) << "Clone without USD should return error";
}
