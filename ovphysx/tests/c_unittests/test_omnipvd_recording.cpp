// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// End-to-end tests for OmniPVD OVD recording via ovphysx typed config.
//
// OmniPVD recording is initialized during createPhysics() which runs inside
// ovphysx_create_instance(). Config must therefore be passed via create_args.
// These tests use standalone TEST() (not PhysXTestFixture) to control the
// full instance lifecycle.

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <filesystem>

using namespace test_utils;

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Count files matching *_rec.ovd in the given directory.
static int count_ovd_files(const std::string& dir)
{
    int count = 0;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec))
    {
        const auto name = entry.path().filename().string();
        if (name.size() > 8 && name.substr(name.size() - 8) == "_rec.ovd")
            ++count;
    }
    return count;
}

// Create an instance, load USD, step, destroy. Returns the number of
// *_rec.ovd files found in output_dir after destruction.
static ::testing::AssertionResult run_recording_workflow(
    const std::string& output_dir,
    bool enable_pvd,
    int& out_ovd_count)
{
    out_ovd_count = 0;

    // Configure OmniPVD via typed config entries passed at instance creation.
    ovphysx_config_entry_t config[] = {
        ovphysx_config_entry_omnipvd_ovd_recording_directory(make_ovx_string(output_dir.c_str())),
        ovphysx_config_entry_omnipvd_output_enabled(enable_pvd),
    };

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = config;
    args.config_entry_count = 2;
    ovphysx_handle_t handle = 0;
    ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
    if (r.status != OVPHYSX_API_SUCCESS || handle == 0)
        return ::testing::AssertionFailure() << "Failed to create PhysX instance";

    if (!attach_usd_with_ovstage(handle, "tests/data/simple_physics_scene.usda"))
    {
        ovphysx_destroy_instance(handle);
        return ::testing::AssertionFailure() << "Failed to load USD scene";
    }

    // Step simulation.
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i)
    {
        r = ovphysx_step_sync(handle, dt);
        if (r.status != OVPHYSX_API_SUCCESS)
        {
            destroy_ovstage_test_attachments(handle);
            ovphysx_destroy_instance(handle);
            return ::testing::AssertionFailure() << "Simulation step " << i << " failed";
        }
    }

    // Destroy instance — this triggers writeOutOmniPVDFile() which renames
    // tmp.ovd to <timestamp>_rec.ovd.
    destroy_ovstage_test_attachments(handle);
    ovphysx_destroy_instance(handle);

    out_ovd_count = count_ovd_files(output_dir);
    return ::testing::AssertionSuccess();
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Positive path: recording directory + enable → at least one non-empty .ovd file.
// OmniPVD is disabled on aarch64 (#if !CARB_AARCH64 in Setup.cpp).
TEST(OmniPvdRecording, OvdFileProducedWhenEnabled)
{
#if defined(__aarch64__) || defined(_M_ARM64)
    GTEST_SKIP() << "OmniPVD is not supported on aarch64";
#endif
    auto tmp_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_enabled";
    fs::create_directories(tmp_dir);
    // Clean up any leftover files from previous runs.
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(tmp_dir, ec))
        fs::remove(entry.path());

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(tmp_dir.string(), /*enable_pvd=*/true, ovd_count));
    EXPECT_GE(ovd_count, 1)
        << "Expected at least one *_rec.ovd file in " << tmp_dir;

    // Verify the file is non-empty.
    for (const auto& entry : fs::directory_iterator(tmp_dir, ec))
    {
        const auto name = entry.path().filename().string();
        if (name.size() > 8 && name.substr(name.size() - 8) == "_rec.ovd")
        {
            EXPECT_GT(fs::file_size(entry.path()), 0u)
                << "OVD file should be non-empty: " << entry.path();
        }
    }

    // Cleanup.
    fs::remove_all(tmp_dir, ec);
}

// Negative path: recording disabled → no .ovd files.
TEST(OmniPvdRecording, NoOvdFileWhenDisabled)
{
    auto tmp_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_disabled";
    fs::create_directories(tmp_dir);
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(tmp_dir, ec))
        fs::remove(entry.path());

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(tmp_dir.string(), /*enable_pvd=*/false, ovd_count));
    EXPECT_EQ(ovd_count, 0)
        << "Expected no *_rec.ovd files when recording is disabled";

    fs::remove_all(tmp_dir, ec);
}

// Negative path: enable with empty directory string → no crash, no .ovd file.
TEST(OmniPvdRecording, EmptyDirectoryDoesNotCrash)
{
    ovphysx_config_entry_t config[] = {
        ovphysx_config_entry_omnipvd_ovd_recording_directory(OVPHYSX_LITERAL("")),
        ovphysx_config_entry_omnipvd_output_enabled(true),
    };

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entries = config;
    args.config_entry_count = 2;
    ovphysx_handle_t handle = 0;
    ovphysx_result_t r = ovphysx_create_instance(&args, &handle);
    // Instance creation should succeed even with misconfigured OmniPVD.
    ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);

    r = ovphysx_step_sync(handle, 1.0f / 60.0f);

    ovphysx_destroy_instance(handle);
    // If we got here without crashing, the test passes.
}

// Directory auto-creation: a non-existent nested path is created by the runtime.
// OmniPVD is disabled on aarch64 (#if !CARB_AARCH64 in Setup.cpp).
TEST(OmniPvdRecording, DirectoryAutoCreated)
{
#if defined(__aarch64__) || defined(_M_ARM64)
    GTEST_SKIP() << "OmniPVD is not supported on aarch64";
#endif
    auto base_dir = fs::temp_directory_path() / "ovphysx_pvd_cpp_test_autocreate";
    auto nested_dir = base_dir / "sub" / "recordings";

    // Ensure the directory does NOT exist before the test.
    std::error_code ec;
    fs::remove_all(base_dir, ec);
    ASSERT_FALSE(fs::exists(nested_dir));

    int ovd_count = 0;
    ASSERT_TRUE(run_recording_workflow(nested_dir.string(), /*enable_pvd=*/true, ovd_count));

    EXPECT_TRUE(fs::is_directory(nested_dir))
        << "Expected the runtime to auto-create " << nested_dir;
    EXPECT_GE(ovd_count, 1)
        << "Expected at least one *_rec.ovd file in auto-created directory";

    fs::remove_all(base_dir, ec);
}
