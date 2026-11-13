// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-DIRECTGPU-DIAG-001
 * @covers AC-1 AC-3
 */

#include <gtest/gtest.h>

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h> // ovphysx_config_entry_carbonite, create-args helpers
#include "global_test_environment.h" // ensureSharedCpuInstance (logging-ready process)

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

// Instance creation fires the effective-DirectGPU line (ovphysx.cpp), after create-config is applied
// and before plugins load. It needs no CUDA device, so these cases are not GPU-gated. AC-2 (the
// bootstrap-snapshot wording) is inspection-verified: that line fires once per process before the
// shared instance exists, so no test can observe it.
namespace
{
struct LogCapture
{
    std::mutex mutex;
    std::vector<std::string> messages;
};

void captureCallback(ovphysx_log_level_t, ovphysx_string_t message, ovphysx_string_t, double, void* user)
{
    LogCapture* cap = static_cast<LogCapture*>(user);
    std::lock_guard<std::mutex> guard(cap->mutex);
    cap->messages.emplace_back(message.ptr ? message.ptr : "", message.length);
}

// Capture the INFO records emitted while creating one instance, with the suppressReadback override on
// `directGpu`. Logging is readied by the shared CPU instance first, so the callback is live before the
// instance under test is created; the shared instance's own records are not captured (the callback is
// installed after it exists).
std::vector<std::string> captureCreationLogs(bool directGpu)
{
    EXPECT_TRUE(ensureSharedCpuInstance());

    const uint32_t previousLevel = ovphysx_get_log_level();
    (void)ovphysx_set_log_level(OVPHYSX_LOG_INFO);

    LogCapture capture;
    EXPECT_EQ(ovphysx_set_log_callback(OVPHYSX_LOG_INFO, nullptr, captureCallback, &capture).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    // Set the override explicitly both ways. /physics/suppressReadback is a process-global carb
    // setting that instance destroy does not revert, so a prior instance (here or in another suite)
    // could leave it set true; an explicit value keeps each case self-contained. The effective line
    // reads the same for an explicit false as for an unset default.
    ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"),
                                       directGpu ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false")),
    };
    args.config_entries = entries;
    args.config_entry_count = sizeof(entries) / sizeof(entries[0]);

    ovphysx_instance_handle_t handle = 0;
    const ovphysx_result_t created = ovphysx_create_instance_with_args(&args, &handle);
    EXPECT_EQ(created.status, OVPHYSX_API_SUCCESS);
    if (handle != 0)
        ovphysx_destroy_instance(handle);

    (void)ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
    (void)ovphysx_set_log_level(previousLevel);

    std::lock_guard<std::mutex> guard(capture.mutex);
    return capture.messages;
}

bool anyContains(const std::vector<std::string>& messages, const std::string& needle)
{
    return std::any_of(messages.begin(), messages.end(),
                       [&](const std::string& m) { return m.find(needle) != std::string::npos; });
}

int countContains(const std::vector<std::string>& messages, const std::string& needle)
{
    return static_cast<int>(std::count_if(messages.begin(), messages.end(),
                                          [&](const std::string& m) { return m.find(needle) != std::string::npos; }));
}
} // namespace

// AC-1/AC-3: the opt-in instance reports the effective line as enabled.
TEST(DirectGpuStartupDiagnostic, OptInReportsEffectiveEnabled)
{
    const std::vector<std::string> messages = captureCreationLogs(/*directGpu=*/true);
    EXPECT_TRUE(anyContains(messages, "/physics/suppressReadback=true after create-config"));
    EXPECT_TRUE(anyContains(messages, "DirectGPU enabled"));
    EXPECT_EQ(countContains(messages, "after create-config (process-wide)"), 1);
}

// AC-1/AC-3: a non-DirectGPU instance still reports a usable effective line, disabled.
TEST(DirectGpuStartupDiagnostic, NoOptInReportsEffectiveDisabled)
{
    const std::vector<std::string> messages = captureCreationLogs(/*directGpu=*/false);
    EXPECT_TRUE(anyContains(messages, "/physics/suppressReadback=false after create-config"));
    EXPECT_TRUE(anyContains(messages, "DirectGPU disabled"));
    EXPECT_EQ(countContains(messages, "after create-config (process-wide)"), 1);
}
