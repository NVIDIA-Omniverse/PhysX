// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause


#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysx/version.h"
#include "ovphysxTestHelpers.h"
#include "global_test_environment.h"
#include "test_utilities.h"
#include <iostream>

// Note: This test is also used to trigger extension pre-caching during build.
TEST_F(PhysXTestFixture, PhysXInstantiation) {
    // Fixture does everything.
    EXPECT_TRUE(true);
}

// ============================================================================
// Version API Tests
// ============================================================================

// ovphysx_get_version fills all three components without crashing.
TEST(VersionAPI, GetVersionFillsAllComponents)
{
    uint32_t major = 0xFFFFFFFF, minor = 0xFFFFFFFF, patch = 0xFFFFFFFF;
    ovphysx_get_version(&major, &minor, &patch);

    // At least one component must have been written (sentinel values cleared)
    EXPECT_TRUE(major != 0xFFFFFFFF || minor != 0xFFFFFFFF || patch != 0xFFFFFFFF)
        << "ovphysx_get_version must update at least one out parameter";

    std::cout << "  ovphysx version: " << major << "." << minor << "." << patch << std::endl;
}

// ovphysx_get_version_string returns a non-null, non-empty string containing a digit.
TEST(VersionAPI, GetVersionStringIsNonEmpty)
{
    const char* version_str = ovphysx_get_version_string();
    ASSERT_NE(version_str, nullptr) << "ovphysx_get_version_string must return non-null";

    const ovphysx_string_t version = ovphysx_cstr(version_str);
    const size_t len = version.length;
    EXPECT_GT(len, 0u) << "Version string must not be empty";

    bool has_digit = false;
    for (size_t i = 0; i < len; ++i)
    {
        if (version_str[i] >= '0' && version_str[i] <= '9')
        {
            has_digit = true;
            break;
        }
    }
    EXPECT_TRUE(has_digit) << "Version string must contain at least one digit";

    std::cout << "  ovphysx version string: " << version_str << std::endl;
}

// Runtime and compile-time versions must agree (prevents stale-header issues).
TEST(VersionAPI, RuntimeVersionMatchesCompileTimeVersion)
{
    uint32_t rt_major = 0, rt_minor = 0, rt_patch = 0;
    ovphysx_get_version(&rt_major, &rt_minor, &rt_patch);

    EXPECT_EQ(rt_major, static_cast<uint32_t>(OVPHYSX_VERSION_MAJOR))
        << "Runtime major version must match compile-time OVPHYSX_VERSION_MAJOR";
    EXPECT_EQ(rt_minor, static_cast<uint32_t>(OVPHYSX_VERSION_MINOR))
        << "Runtime minor version must match compile-time OVPHYSX_VERSION_MINOR";
    EXPECT_EQ(rt_patch, static_cast<uint32_t>(OVPHYSX_VERSION_PATCH))
        << "Runtime patch version must match compile-time OVPHYSX_VERSION_PATCH";
}

TEST(CreateInstanceValidation, RejectsNullConfigEntriesWithNonzeroCount)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.config_entry_count = 1;
    const ovphysx_handle_t sentinel = UINT64_MAX;
    ovphysx_handle_t handle = sentinel;

    const ovphysx_result_t result = ovphysx_create_instance(&args, &handle);

    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(handle, sentinel);
}

// ============================================================================
// active_cuda_gpus Parse Validation
// ============================================================================
// These tests exercise the active_cuda_gpus parser via the public C API.
// Parse errors are returned before runtime initialization, so no GPU is needed.

// Exercises the active_cuda_gpus parser without requiring CUDA. The parser runs
// early (before attach), so this covers parse validation. Invalid inputs fail at
// create time; valid inputs proceed through instance creation.
static ovphysx_api_status_t try_create_with_gpus(const char* gpus)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.active_cuda_gpus = ovphysx_cstr(gpus);
    ovphysx_handle_t handle = 0;
    ovphysx_result_t result = ovphysx_create_instance(&args, &handle);
    if (result.status == OVPHYSX_API_SUCCESS && handle)
        ovphysx_destroy_instance(handle);
    return result.status;
}

TEST(ActiveCudaGpusParsing, InvalidFormat_NonNumeric)
{
    EXPECT_EQ(try_create_with_gpus("abc"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_DuplicateOrdinals)
{
    EXPECT_EQ(try_create_with_gpus("0,0"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_NegativeMixedWithPositive)
{
    EXPECT_EQ(try_create_with_gpus("-1,0"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_EmptyBetweenCommas)
{
    EXPECT_EQ(try_create_with_gpus("0,,1"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_LetterAfterDigit)
{
    EXPECT_EQ(try_create_with_gpus("0a"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_TrailingComma)
{
    EXPECT_EQ(try_create_with_gpus("0,"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_OrdinalOverflow)
{
    EXPECT_EQ(try_create_with_gpus("9999999999"), OVPHYSX_API_INVALID_ARGUMENT);
}

TEST(ActiveCudaGpusParsing, InvalidFormat_NegativeOtherThanMinusOne)
{
    EXPECT_EQ(try_create_with_gpus("-2"), OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(try_create_with_gpus("-5"), OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(try_create_with_gpus("-100"), OVPHYSX_API_INVALID_ARGUMENT);
}

// Valid formats must pass parsing and succeed with CPU device.
TEST(ActiveCudaGpusParsing, ValidFormat_SingleOrdinalZero)
{
    EXPECT_EQ(try_create_with_gpus("0"), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusParsing, ValidFormat_MinusOne_PhysXAutoSelect)
{
    EXPECT_EQ(try_create_with_gpus("-1"), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusParsing, ValidFormat_SingleDigitOrdinal)
{
    EXPECT_EQ(try_create_with_gpus("2"), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusParsing, ValidFormat_WhitespaceAroundOrdinals)
{
    EXPECT_EQ(try_create_with_gpus(" 0 "), OVPHYSX_API_SUCCESS);
}

TEST(ActiveCudaGpusAttachTest, DirectOvstageAttachPropagatesExplicitOrdinal)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    args.active_cuda_gpus = OVPHYSX_LITERAL("0");
    const ovphysx_config_entry_t staleMultiGpuMode =
        ovphysx_config_entry_scene_multi_gpu_mode(1);
    args.config_entries = &staleMultiGpuMode;
    args.config_entry_count = 1;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    const ovphysx_result_t createResult = ovphysx_create_instance(&args, &handle);
    if (createResult.status != OVPHYSX_API_SUCCESS)
    {
        ADD_FAILURE() << "Failed to create an instance with active_cuda_gpus=0";
        return;
    }

    int32_t attachCudaSelector = 0;
    if (!ovphysx_get_attach_cuda_selector_for_test_internal(&attachCudaSelector))
    {
        ADD_FAILURE() << "Attach-time CUDA selector is unavailable";
        EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
        return;
    }
    EXPECT_EQ(attachCudaSelector, -1);

    int32_t multiGpuMode = -1;
    EXPECT_EQ(ovphysx_get_global_config_int32(
                  OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, &multiGpuMode).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(multiGpuMode, 1);

    const bool attached = test_utils::attach_usd_with_ovstage(
        handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda");
    EXPECT_TRUE(attached);
    if (attached)
    {
        EXPECT_TRUE(ovphysx_get_attach_cuda_selector_for_test_internal(&attachCudaSelector));
        EXPECT_EQ(attachCudaSelector, 0);
        EXPECT_EQ(ovphysx_get_global_config_int32(
                      OVPHYSX_CONFIG_SCENE_MULTI_GPU_MODE, &multiGpuMode).status,
                  OVPHYSX_API_SUCCESS);
        EXPECT_EQ(multiGpuMode, 0);
    }

    if (attached)
    {
        EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(handle));
    }
    EXPECT_EQ(ovphysx_destroy_instance(handle).status, OVPHYSX_API_SUCCESS);
}
