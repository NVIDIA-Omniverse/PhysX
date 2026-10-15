// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-HOST-EXIT-001
 * @covers AC-1 AC-2 AC-3
 */

#include <gtest/gtest.h>
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "test_utilities.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{
void hostExitHandler()
{
    std::fputs("OVPHYSX_HOST_EXIT_OK\n", stderr);
}

void checkSuccess(ovphysx_result_t result)
{
    if (result.status != OVPHYSX_API_SUCCESS)
        std::_Exit(99);
}

void prepareHostExit(FILE* file, bool attachStage)
{
#if defined(_WIN32)
    // Windows retains Carbonite's terminating ExitProcess hook for safe
    // dependency DLL teardown. Windows hosts must flush buffered output explicitly.
    std::fflush(file);
#else
    // OVStage has an independent terminating exit handler. The stage cases
    // check safe dependency teardown; only ovphysx-only cases check implicit flush.
    if (attachStage)
        std::fflush(file);
#endif
    if (attachStage && std::atexit(hostExitHandler) != 0)
        std::_Exit(98);
}

void checkHostExit(int cycles, bool attachStage = false)
{
    // Re-exec a fresh host, even if the parent test process already has workers.
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    const auto path = std::filesystem::path(testing::TempDir()) /
        ("ovphysx-host-exit-" + std::to_string(cycles) + (attachStage ? "-stage.txt" : ".txt"));
    std::filesystem::remove(path);
    ASSERT_EXIT(
        {
            if (!attachStage && std::atexit(hostExitHandler) != 0)
                std::_Exit(98);
            FILE* file = std::fopen(path.string().c_str(), "wb");
            static char buffer[4096];
            if (!file || std::setvbuf(file, buffer, _IOFBF, sizeof(buffer)) != 0)
                std::_Exit(97);
            std::fputs("payload\n", file);
            for (int i = 0; i < cycles; ++i)
            {
                checkSuccess(ovphysx_initialize());
                checkSuccess(ovphysx_set_global_config(
                    OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, ovphysx_config_value_bool(true)));
                checkSuccess(ovphysx_set_global_config(
                    OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(4)));
                ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
                checkSuccess(ovphysx_create_instance(&handle));
                if (attachStage)
                {
                    if (!test_utils::attach_usd_with_ovstage(
                            handle, OVPHYSX_SOURCE_DIR "/tests/data/minimal_scene.usda") ||
                        !test_utils::destroy_ovstage_test_attachments(handle))
                        std::_Exit(96);
                }
                checkSuccess(ovphysx_destroy_instance(handle));
                checkSuccess(ovphysx_shutdown());
            }
            prepareHostExit(file, attachStage);
            std::exit(23);
        },
        testing::ExitedWithCode(23), "OVPHYSX_HOST_EXIT_OK");
    std::ifstream file(path, std::ios::binary);
    const std::string contents((std::istreambuf_iterator<char>(file)), {});
    EXPECT_EQ(contents, "payload\n");
    file.close();
    std::filesystem::remove(path);
}
} // namespace

TEST(HostExit, ExplicitShutdownPreservesHostCleanup)
{
    checkHostExit(1);
}

TEST(HostExit, RepeatedLifecyclesPreserveHostCleanup)
{
    checkHostExit(3);
}

TEST(HostExit, StageShutdownPreservesCleanExit)
{
    checkHostExit(1, true);
}

TEST(HostExit, RepeatedStageLifecyclesPreserveCleanExit)
{
    checkHostExit(3, true);
}
